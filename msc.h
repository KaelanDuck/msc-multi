#pragma once
#include <libopencm3/cm3/common.h>
#include <libopencm3/usb/usbd.h>
#include <stdint.h>
#include <stdbool.h>

BEGIN_DECLS

// required for implementing interface descriptors
// use same naming as libopencm3
#define USB_CLASS_MSC (0x08)
#define USB_MSC_SUBCLASS_SCSI (0x06)
#define USB_MSC_PROTOCOL_BBB (0x50)

typedef enum {
    MSC_RESULT_OK = 0,
    MSC_RESULT_READ_ONLY,
    MSC_RESULT_IO_ERROR,
    MSC_RESULT_INVALID_PARAMETER
} msc_result;

struct scsi_inquiry_data {
    char vendor_id[8];
    char product_id[16];
    char product_revision[4];
};

struct device_query_data {
    uint32_t num_blocks;
    bool read_only;
    bool not_present;
};

/**
 * @brief initializes the MSC interface
 * 
 * @param device handle to usbd device
 * @param interface_id interface id as per configuration descriptor. Usually 0
 * @param ep_in IN endpoint
 * @param ep_in_size IN endpoint size
 * @param ep_out OUT endpoint
 * @param ep_out_size OUT endpoint size
 * @param num_luns number of LUNs (logical units) in device
 * @param inquiry_data data to send for scsi inquiry, one array entry per lun. Can be NULL to use generic data
 * @param num_blocks returns the number of blocks in a given lun
 * @param read_block reads one block from a device
 * @param write_block writes one block to a device. Can be NULL for a read-only device
 * @param is_read_only tests if a lun should be read only. Can be NULL (write_block pointer will be tested)
 * @param is_present tests if a lun should be present. Can be NULL for always present
 */
void msc_init_multi(
    usbd_device *device,
    uint8_t interface_id,
    uint8_t ep_in, uint8_t ep_in_size,
    uint8_t ep_out, uint8_t ep_out_size,
    int num_luns,
    const struct scsi_inquiry_data *inquiry_data,
    uint32_t   (*num_blocks) (int lun),
    msc_result (*read_block) (int lun, uint32_t lba, uint8_t *copy_to),
    msc_result (*write_block) (int lun, uint32_t lba, const uint8_t *copy_from),
    bool       (*is_read_only) (int lun),
    bool       (*is_present) (int lun)
);


END_DECLS