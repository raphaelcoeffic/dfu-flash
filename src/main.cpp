#include <stdio.h>

#include <algorithm>
#include <fstream>
#include <optional>

#include "dfu.h"

#define BYTES_PER_KILOBYTE 1024
#define PBSTR "############################################################"
#define PBWIDTH 60


void list_devices(std::optional<uint16_t> vendor = std::nullopt,
                  std::optional<uint16_t> product = std::nullopt)
{
  dfu::init();
  auto devs = dfu::find_devices(vendor, product);

  for (auto dfu : devs) {
    printf("Bus %d Device %03d: ID %04x:%04x\n", dfu.bus_number(),
           dfu.device_address(), dfu.vendor, dfu.product);

    for (auto intf : dfu.interfaces) {
      for (auto seg : intf.memory_layout()) {
        auto pages = (seg.end - seg.start) / seg.pagesize;
        auto page_size = seg.pagesize;

        const char* page_char = " ";
        if (page_size >= BYTES_PER_KILOBYTE) {
          page_size /= BYTES_PER_KILOBYTE;
          page_char = "K";
        }

        printf("    %d: 0x%08X %2d pages of %4d%s bytes (%s%s%s)\n",
               intf.alt_setting, seg.start,
               pages, page_size, page_char, seg.readable() ? "r" : "",
               seg.writable() ? "w" : "", seg.erasable() ? "e" : "");
      }
    }
  }
  dfu::finish();
}

using intf_segment = std::pair<dfu::dfu_interface, dfu::dfu_memory_layout>;

std::optional<intf_segment> find_interface_segment(
    const std::vector<dfu::dfu_interface>& interfaces, uint32_t start_address,
    uint32_t end_address)
{
  for (auto intf : interfaces) {
    auto layout = intf.memory_layout();
    dfu::dfu_memory_layout out_layout;

    std::copy_if(layout.begin(), layout.end(), std::back_inserter(out_layout),
                 [&](const dfu::mem_segment& seg) {
                   return start_address <= seg.start &&
                          seg.start <= end_address;
                 });

    if (!out_layout.empty()) return {{intf, std::move(out_layout)}};
  }

  return {};
}

int download(dfu::dfu_device& dfu, const uint8_t* data, size_t data_len,
             uint32_t start_addr)
{
  int ret = 0;
  auto dfu_desc = dfu.get_dfu_descriptor();
  uint16_t bcdDFUVersion = dfu_desc.bcdDFUVersion;
  if (bcdDFUVersion == DFUSE_VERSION_NUMBER) {
    printf("DFuSe device detected (xfer_size=%d)\n", dfu_desc.wTransferSize);
  }

  uint32_t end_addr = start_addr + data_len - 1;
  auto opt_intf_seg =
      find_interface_segment(dfu.interfaces, start_addr, end_addr);
  if (!opt_intf_seg.has_value()) {
    fprintf(stderr, "Could not find a suitable set of address segments.\n");
    return -1;
  }

  auto intf_seg{opt_intf_seg.value()};
  auto intf = intf_seg.first;
  printf("DFuSe interface [%d:%d]\n", intf.interface, intf.alt_setting);

  auto connection = intf.connect();
  if (!connection) {
    fprintf(stderr, "Could not claim DFU device\n");
  } else {
    dfu::dfu_status st;
    ret = connection->get_status(st);
    if (ret == 0 && st.status == DFU_STATE_DFU_ERROR) {
      if (connection->clear_status() == LIBUSB_SUCCESS) {
        ret = connection->get_status(st);
      }
    }

    if (ret == 0 && st.status != 0) {
      fprintf(stderr, "Error while querying device state: %d\n", st.status);
      return -1;
    }
  }

  const auto& segments = intf_seg.second;
  for (const auto& seg : segments) {
    // compute erase start & end in this segment
    uint32_t erase_start = std::max(start_addr, seg.start);
    uint32_t erase_end = std::min(end_addr, seg.end);

    // verify page alignement
    if ((start_addr > seg.start) &&
        ((erase_start - seg.start) % seg.pagesize != 0)) {
      fprintf(stderr, "Start address must be at a page boundary\n");
      return -1;
    }

    auto pages = (erase_end - erase_start + seg.pagesize) / seg.pagesize;
    printf("  Erasing %d pages @ 0x%08X ", pages, seg.start);
    fflush(stdout);

    for (uint32_t page_addr = erase_start; page_addr < erase_end;
         page_addr += seg.pagesize) {

      int status = connection->dfuse_page_erase(page_addr);
      if (status < 0) {
        printf(" ❌\n");
        return status;
      } else {
        printf(".");
        fflush(stdout);
      }
    }

    printf(" %s\n", "✅");
  }

  auto wTransferSize = dfu_desc.wTransferSize;
  size_t bytes_downloaded = 0;

  int status = 0;
  while(bytes_downloaded < data_len) {
    auto xfer_size = std::min((size_t)wTransferSize, data_len - bytes_downloaded);
    auto xfer_data = (unsigned char*)(data + bytes_downloaded);
    status = connection->download(start_addr + bytes_downloaded, xfer_data, xfer_size);
    if (status < 0) return status;

    bytes_downloaded += xfer_size;
    double percentage = (double)bytes_downloaded / (double)data_len;
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
     
    printf("\r  Flashing [%.*s%*s]", lpad, PBSTR, rpad, "");
    fflush(stdout);
  }
  printf("\n");

  // leave DFU mode
  printf("Leave DFU mode\n");
  connection->dfuse_leave(start_addr);

  return ret;
}

int upload(dfu::dfu_device& connection, const uint8_t* data, size_t data_len,
           uint32_t start_address)
{
  return 0;
}

void usage(const char* progname)
{
  fprintf(stderr, "Usage: %s {list | write | read} [filename]\n", progname);
}

int read_file(const char* filename, std::vector<char>& buffer)
{
  std::ifstream file(filename, std::ios::binary | std::ios::ate);
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  buffer.resize(size);
  if (!file.read(buffer.data(), size)) {
    fprintf(stderr, "Could not read '%s'\n", filename);
    return -1;
  }

  return 0;
}

int main(int argc, char** argv)
{
  if (argc < 2) {
    usage(argv[0]);
    return 1;
  }

  bool write;
  if (!strcmp(argv[1], "list")) {
    list_devices();
    return 0;
  }

  if (argc < 3) {
    fprintf(stderr, "Missing filename\n");
    return 1;
  }

  const char* filename = argv[2];
  if (!strcmp(argv[1], "write")) {
    write = true;
  } else if (!strcmp(argv[1], "read")) {
    write = false;
  } else {
    fprintf(stderr, "Invalid command '%s'", argv[1]);
    return 1;
  }

  dfu::init();
  auto devs = dfu::find_devices(std::nullopt, std::nullopt);
  auto n_devices = devs.size();

  int ret = 1;
  if (n_devices < 1) {
    fprintf(stderr, "No DFU device found.\n");
  } else if (n_devices > 1) {
    fprintf(stderr, "More than one DFU devices found.\n");
  } else {
    std::vector<char> fw;
    if (read_file(filename, fw) != 0) {
      ret = 1;
    } else {
      auto dev = devs[0];
      auto data = (const uint8_t*)fw.data();
      auto len = fw.size();
      uint32_t addr = 0x08000000;
      ret =
          write ? download(dev, data, len, addr) : upload(dev, data, len, addr);
      if (ret) {
        fprintf(stderr, "Error: %d\n", ret);
      }
    }
  }

  dfu::finish();
  return ret;
}
