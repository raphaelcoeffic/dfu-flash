#include "dfu.h"

#include <string.h>
#include <unistd.h>

#include <regex>

#define TIMEOUT_MS 5000

#define DFU_DESCRIPTOR_LEN 9
#define DFU_DESCRIPTOR_ID 0x21

#define MAX_DESC_STR_LEN 253

// #define USB_REQUEST_TYPE_SEND 0x21
#define USB_REQUEST_TYPE_SEND \
  LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE

#define USB_REQUEST_TYPE_RECV 0xA1

#define DFU_CMD_DOWNLOAD 1
#define DFU_CMD_GETSTATUS 3
#define DFU_CMD_CLRSTATUS 4
#define DFU_STATE_LEN 6

#define DFUSE_CMD_ADDR 0x21
#define DFUSE_CMD_ERASE 0x41

namespace dfu
{

void _free_device_list(libusb_device** devs)
{
  if (devs) libusb_free_device_list(devs, 1);
}

class device_list
{
  std::shared_ptr<libusb_device*> devs;

 public:
  device_list() {}
  device_list(libusb_device** devs) : devs{devs, _free_device_list} {}

  bool empty() const { return !devs; }
  libusb_device** get() const { return devs.get(); }
};

device_list get_device_list(libusb_context* ctx)
{
  libusb_device** devs = nullptr;
  ssize_t cnt = libusb_get_device_list(ctx, &devs);
  if (cnt < 0) return {};
  return {devs};
}

context::context(libusb_context* ctx) : ctx(ctx, libusb_exit) {}

std::optional<context> init()
{
#if LIBUSB_API_VERSION >= 0x0100010A
#define _init_libusb(ctx) libusb_init_context(ctx, nullptr, 0)
#else
#define _init_libusb(ctx) libusb_init(ctx)
#endif
  libusb_context* ctx = nullptr;
  if (_init_libusb(&ctx) < 0) {
    return {};
  }
  return {{ctx}};
}

dfu_devices context::find_devices(std::optional<uint16_t> vendor,
                                  std::optional<uint16_t> product)
{
  dfu_devices dfu_devs;
  auto devs = get_device_list(ctx.get());
  if (devs.empty()) return dfu_devs;

  unsigned int dev_idx = 0;
  libusb_device* dev_ptr = nullptr;

  while ((dev_ptr = devs.get()[dev_idx++]) != nullptr) {
    auto dev = device{dev_ptr};
    auto desc = dev.descriptor();

    if (vendor.has_value() && vendor.value() != desc.idVendor) continue;
    if (product.has_value() && product.value() != desc.idProduct) continue;

    dfu_device dfu{dev, desc.idVendor, desc.idProduct};
    for (auto cfg : dev.configurations()) {
      for (int i = 0; i < cfg->bNumInterfaces; i++) {
        const auto& intf = cfg->interface[i];
        for (int alt = 0; alt < intf.num_altsetting; alt++) {
          const auto& intf_desc = intf.altsetting[alt];
          if (intf_desc.bInterfaceClass == 0xFE &&
              intf_desc.bInterfaceSubClass == 1) {
            dfu.interfaces.emplace_back(dfu_interface{
                dev, cfg->bConfigurationValue, intf_desc.bInterfaceNumber,
                intf_desc.bAlternateSetting, intf_desc.iInterface});
          }
        }
      }
    }

    if (!dfu.interfaces.empty()) {
      dfu_devs.emplace_back(std::move(dfu));
    }
  }

  return dfu_devs;
}

device::device(libusb_device* dev) :
    dev(libusb_ref_device(dev), libusb_unref_device)
{
}

libusb_device_descriptor device::descriptor()
{
  libusb_device_descriptor desc;
  libusb_get_device_descriptor(dev.get(), &desc);
  return desc;
}

std::vector<configuration_ptr> device::configurations()
{
  auto bNumConfigurations = descriptor().bNumConfigurations;
  std::vector<configuration_ptr> confs;
  for (int i = 0; i < bNumConfigurations; i++) {
    libusb_config_descriptor* cfg_desc = nullptr;
    if (libusb_get_config_descriptor(dev.get(), i, &cfg_desc) == 0) {
      confs.emplace_back(cfg_desc, libusb_free_config_descriptor);
    }
  }
  return confs;
}

configuration_ptr device::configuration(uint8_t cfg)
{
  libusb_config_descriptor* cfg_desc = nullptr;
  if (libusb_get_config_descriptor(dev.get(), cfg, &cfg_desc) != 0) {
    return {};
  }
  return {cfg_desc, libusb_free_config_descriptor};
}

void _close_handle(libusb_device_handle* h)
{
  libusb_close(h);
}

device_handle device::open()
{
  libusb_device_handle* dev_handle;
  if (libusb_open(dev.get(), &dev_handle) != 0) {
    return {};
  }
  return {dev_handle, _close_handle};
}

static dfu_descriptor _make_dfu_descriptor(const uint8_t* dfu_desc)
{
  uint16_t wDetachTimeOut = dfu_desc[4] << 8 | dfu_desc[3];
  uint16_t wTransferSize = dfu_desc[6] << 8 | dfu_desc[5];
  uint16_t bcdDFUVersion = dfu_desc[8] << 8 | dfu_desc[7];
  uint8_t bmAttributes = dfu_desc[2];

  return {wDetachTimeOut, wTransferSize, bcdDFUVersion, bmAttributes};
}

dfu_descriptor dfu_device::get_dfu_descriptor()
{
  for (auto dfu_intf : interfaces) {
    auto cfg = dev.configuration(dfu_intf.config - 1);
    if (!cfg) continue;

    if (dfu_intf.interface >= cfg->bNumInterfaces) continue;
    const auto& intf = cfg->interface[dfu_intf.interface];

    if (dfu_intf.alt_setting >= intf.num_altsetting) continue;
    const auto& intf_desc = intf.altsetting[dfu_intf.alt_setting];

    if (intf_desc.extra_length != DFU_DESCRIPTOR_LEN ||
        intf_desc.extra[1] != DFU_DESCRIPTOR_ID)
      continue;

    return _make_dfu_descriptor(intf_desc.extra);
  }

  return {};
}

std::string dfu_interface::alt_name()
{
  auto h = dev.open();
  if (!h) return std::string{};

  char name[MAX_DESC_STR_LEN + 1];
  unsigned char* buf = (unsigned char*)name;

  if (libusb_get_string_descriptor_ascii(h.get(), name_idx, buf,
                                         MAX_DESC_STR_LEN) <= 0) {
    return std::string{};
  }

  return std::string{name};
}

dfu_memory_layout dfu_interface::memory_layout()
{
  dfu_memory_layout layout;
  auto mem_layout_str = alt_name();

  std::smatch m;
  std::regex r("[^/]*\\/(0x[\\da-fA-F]+)U?\\/(.*)");
  if (!std::regex_match(mem_layout_str, m, r)) return layout;

  uint32_t start_addr = std::stoul(m[1].str(), nullptr, 0);

  auto segments = m[2].str();
  std::regex sr("(\\d+)\\*(\\d+)([KMB])([a-g])(?:,|$)");
  auto seg_begin = std::sregex_iterator(segments.begin(), segments.end(), sr);
  auto seg_end = std::sregex_iterator();

  for (auto seg = seg_begin; seg != seg_end; ++seg) {
    std::smatch m = *seg;

    uint32_t pages = std::stoul(m[1].str());
    uint32_t page_size = std::stoul(m[2].str());
    auto page_mul = m[3].str();

    if (page_mul == "K")
      page_size *= 1024;
    else if (page_mul == "M")
      page_size *= 1024 * 1024;
    else if (page_mul == "B")
      page_size *= 1;

    uint8_t memtype = m[4].str()[0] & 7;
    uint32_t end_addr = start_addr + pages * page_size;

    auto segment = mem_segment{start_addr, end_addr, page_size, memtype};
    start_addr = end_addr;

    layout.emplace_back(segment);
  }

  return layout;
}

std::shared_ptr<dfu_connection> dfu_interface::connect()
{
  auto h = dev.open();
  if (!h || (libusb_claim_interface(h.get(), interface) < 0)) {
    return {};
  }
  return std::make_shared<dfu_connection>(dev, std::move(h), interface);
}

dfu_connection::~dfu_connection()
{
  libusb_release_interface(h.get(), interface);
}

int dfu_connection::dfu_dnload(int transaction, unsigned char* data,
                               size_t data_len)
{
  int err = libusb_control_transfer(h.get(), USB_REQUEST_TYPE_SEND,
                                    DFU_CMD_DOWNLOAD, transaction, interface,
                                    data, data_len, TIMEOUT_MS);
  if (err < 0) {
    fprintf(stderr, "libusb_control_transfer() returned %s\n",
            libusb_error_name(err));
    return err;
  }

  dfu_status st;
  do {
    err = get_status(st);
  } while (err >= 0 && st.state == DFU_STATE_DFU_DOWNLOAD_BUSY);

  return st.status;
}

int dfu_connection::get_status(dfu_status& st)
{
  uint8_t data[DFU_STATE_LEN];
  memset(&st, 0, sizeof(dfu_status));

  int err =
      libusb_control_transfer(h.get(), USB_REQUEST_TYPE_RECV, DFU_CMD_GETSTATUS,
                              0, interface, data, DFU_STATE_LEN, TIMEOUT_MS);
  if (err < 0) return err;

  st.status = data[0];
  st.poll_timeout = data[3] << 16 | data[2] << 8 | data[1];
  st.state = data[4];

  return 0;
}

int dfu_connection::clear_status()
{
  return libusb_control_transfer(h.get(), USB_REQUEST_TYPE_SEND,
                                 DFU_CMD_CLRSTATUS, 0, interface, nullptr, 0,
                                 TIMEOUT_MS);
}

int dfu_connection::dfuse_page_erase(uint32_t addr)
{
  uint8_t erase_cmd[5];
  erase_cmd[0] = DFUSE_CMD_ERASE;

  // pack address as little endian
  erase_cmd[1] = addr & 0xff;
  erase_cmd[2] = (addr >> 8) & 0xff;
  erase_cmd[3] = (addr >> 16) & 0xff;
  erase_cmd[4] = (addr >> 24) & 0xff;

  int status = dfu_dnload(0, erase_cmd, sizeof(erase_cmd));
  if (status != 0) return -1;

  return 0;
}

int dfu_connection::dfuse_leave(uint32_t addr)
{
  int status = set_address(addr);
  if (status < 0) return status;

  return dfu_dnload(0, nullptr, 0);
}

int dfu_connection::set_address(uint32_t addr)
{
  uint8_t addr_cmd[5];
  addr_cmd[0] = DFUSE_CMD_ADDR;

  // pack address as little endian
  addr_cmd[1] = addr & 0xff;
  addr_cmd[2] = (addr >> 8) & 0xff;
  addr_cmd[3] = (addr >> 16) & 0xff;
  addr_cmd[4] = (addr >> 24) & 0xff;

  int status = dfu_dnload(0, addr_cmd, sizeof(addr_cmd));
  if (status != 0) return -1;

  return 0;
}

int dfu_connection::download(uint32_t addr, const uint8_t* data, size_t len)
{
  int status = set_address(addr);
  if (status < 0) return status;

  return dfu_dnload(2, (unsigned char*)data, len);
}

}  // namespace dfu
