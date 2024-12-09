#include <libusb.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#define DFU_STATE_APP_IDLE 0x00
#define DFU_STATE_APP_DETACH 0x01
#define DFU_STATE_DFU_IDLE 0x02
#define DFU_STATE_DFU_DOWNLOAD_SYNC 0x03
#define DFU_STATE_DFU_DOWNLOAD_BUSY 0x04
#define DFU_STATE_DFU_DOWNLOAD_IDLE 0x05
#define DFU_STATE_DFU_MANIFEST_SYNC 0x06
#define DFU_STATE_DFU_MANIFEST 0x07
#define DFU_STATE_DFU_MANIFEST_WAIT_RESET 0x08
#define DFU_STATE_DFU_UPLOAD_IDLE 0x09
#define DFU_STATE_DFU_ERROR 0x0a

#define DFUSE_VERSION_NUMBER 0x11A

namespace dfu
{

struct dfu_device;

using context_ptr = std::shared_ptr<libusb_context>;
using device_ptr = std::shared_ptr<libusb_device>;
using configuration_ptr = std::shared_ptr<libusb_config_descriptor>;
using device_handle = std::shared_ptr<libusb_device_handle>;
using dfu_devices = std::vector<dfu_device>;

class context
{
  context_ptr ctx;

 public:
  context(context_ptr&& ctx) : ctx(std::move(ctx)) {}
  context(libusb_context* ctx);

  dfu_devices find_devices(std::optional<uint16_t> vendor,
                           std::optional<uint16_t> product);
};

std::optional<context> init();

class device
{
  device_ptr dev;

 public:
  device(device_ptr&& dev) : dev(std::move(dev)) {}
  device(libusb_device* dev);

  libusb_device_descriptor descriptor();
  std::vector<configuration_ptr> configurations();
  configuration_ptr configuration(uint8_t cfg);

  uint8_t bus_number() const { return libusb_get_bus_number(dev.get()); }
  uint8_t device_address() const
  {
    return libusb_get_device_address(dev.get());
  }

  device_handle open();
};

struct mem_segment {
  uint32_t start;
  uint32_t end;
  uint32_t pagesize;
  uint32_t memtype;

  bool readable() { return memtype & 1; }
  bool erasable() { return memtype & 2; }
  bool writable() { return memtype & 4; }
};

using dfu_memory_layout = std::vector<mem_segment>;

class dfu_connection;

struct dfu_interface {
  device dev;

  uint8_t config;
  uint8_t interface;
  uint8_t alt_setting;
  uint8_t name_idx;

  std::string alt_name();
  dfu_memory_layout memory_layout();

  std::shared_ptr<dfu_connection> connect();
};

struct dfu_descriptor {
  uint16_t wDetachTimeOut;
  uint16_t wTransferSize;
  uint16_t bcdDFUVersion;
  uint8_t bmAttributes;
};

struct dfu_device {
  device dev;

  uint16_t vendor;
  uint16_t product;
  std::vector<dfu_interface> interfaces;

  auto bus_number() const { return dev.bus_number(); }
  auto device_address() const { return dev.device_address(); }
  dfu_descriptor get_dfu_descriptor();
};

struct dfu_status {
  uint8_t status;
  uint32_t poll_timeout;
  uint8_t state;
};

class dfu_connection
{
  device dev;
  device_handle h;
  uint8_t interface;

  int dfu_dnload(int transaction, unsigned char* data, size_t data_len);
  int dfu_upload(int transaction, unsigned char* data, size_t data_len);

 public:
  dfu_connection(const device& dev, device_handle&& h, uint8_t interface) :
      dev(dev), h(h), interface(interface)
  {
  }
  ~dfu_connection();

  int reset_state();

  int get_status(dfu_status& st);
  int clear_status();
  int abort();

  int dfuse_page_erase(uint32_t addr);
  int dfuse_leave(uint32_t addr);

  int set_address(uint32_t addr);
  int download(uint32_t addr, const uint8_t* data, size_t len);
  int upload(uint16_t block_nr, uint8_t* data, size_t len);
};

}  // namespace dfu
