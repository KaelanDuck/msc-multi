#include "msc.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>


///////////////////////// DEFINES ////////////////////////////
#define PACKED __attribute__((packed))
#define MIN(x,y) (((x)<(y))?(x):(y))
#define CBW_SIGNATURE (0x43425355)
#define CSW_SIGNATURE (0x53425355)
#define nullptr ((void *)0)

#define CSW_STATUS_OK (0)
#define CSW_STATUS_CHECK_CONDITION (1)

/* for debugging, beware semihosting is very slow */
#if 0
	extern void initialise_monitor_handles(void);
	#define DEBUG_PRINT(...) printf(__VA_ARGS__)
	#define SETUP_DEBUG_PRINT() initialise_monitor_handles()
#else
	#define DEBUG_PRINT(...) (void)0
	#define SETUP_DEBUG_PRINT() (void)0
#endif

// https://www.t10.org/lists/op-num.htm
#define SCSI_TEST_UNIT_READY              (0x00)
#define SCSI_REQUEST_SENSE                (0x03)
#define SCSI_FORMAT_UNIT                  (0x04)
#define SCSI_READ_6                       (0x08)
#define SCSI_WRITE_6                      (0x0A)
#define SCSI_INQUIRY                      (0x12)
#define SCSI_MODE_SENSE_6                 (0x1A)
#define SCSI_SEND_DIAGNOSTIC              (0x1D)
#define SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL (0x1E)
#define SCSI_READ_FORMAT_CAPACITIES       (0x23)
#define SCSI_READ_CAPACITY_10             (0x25)
#define SCSI_READ_10                      (0x28)
#define SCSI_WRITE_10                     (0x2A)
#define SCSI_READ_16                      (0x88)
#define SCSI_WRITE_16                     (0x8A)
#define SCSI_REPORT_LUNS                  (0xA0)

// https://www.t10.org/lists/2sensekey.htm
#define SENSE_KEY_NO_SENSE           (0x00)
#define SENSE_KEY_RECOVERED_ERROR    (0x01)
#define SENSE_KEY_NOT_READY          (0x02)
#define SENSE_KEY_MEDIUM_ERROR       (0x03)
#define SENSE_KEY_HARDWARE_ERROR     (0x04)
#define SENSE_KEY_ILLEGAL_REQUEST    (0x05)
#define SENSE_KEY_UNIT_ATTENTION     (0x06)
#define SENSE_KEY_DATA_PROTECT       (0x07)
#define SENSE_KEY_BLANK_CHECK        (0x08)
#define SENSE_KEY_VENDOR_SPECIFIC    (0x09)
#define SENSE_KEY_COPY_ABORTED       (0x0A)
#define SENSE_KEY_ABORTED_COMMAND    (0x0B)
#define SENSE_KEY_VOLUME_OVERFLOW    (0x0D)
#define SENSE_KEY_MISCOMPARE         (0x0E)
#define SENSE_KEY_COMPLETED          (0x0F)

// https://www.t10.org/lists/asc-num.htm
#define SENSE_ASC_NO_ADDITIONAL_INFO     (0x00)
#define SENSE_ASC_WRITE_FAULT            (0x03)
#define SENSE_ASC_LUN_NOT_READY          (0x04)
#define SENSE_ASC_UNRECOVERED_READ_ERROR (0x11)
#define SENSE_ASC_UNSUPPORTED_COMMAND    (0x20)
#define SENSE_ASC_LBA_OUT_OF_RANGE       (0x21)
#define SENSE_ASC_INVALID_FIELD_IN_CDB   (0x24)
#define SENSE_ASC_LUN_NOT_SUPPORTED      (0x25)
#define SENSE_ASC_WRITE_PROTECTED        (0x27)
#define SENSE_ASC_MEDIUM_NOT_PRESENT     (0x3A)

#define INQUIRY_PERIPHERAL_QUALIFIER_NOT_PRESENT   (1)
#define INQUIRY_PERIPHERAL_QUALIFIER_NOT_SUPPORTED (3)


/////////////////////// STRUCTURES ///////////////////////////

struct CBW {
	uint32_t dCBWSignature;
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[15];
} PACKED;

struct CSW {
	uint32_t dCSWSignature;
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
} PACKED;

struct sense {
	uint8_t error_code : 7;
	bool valid : 1;
	uint8_t segment_number;
	uint8_t sense_key : 4;
	bool : 1;
	bool incorrect_length : 1;
	bool end_of_media : 1;
	bool file_mark : 1;
	uint8_t information[4];
	uint8_t additional_sense_length;
	uint8_t command_specific_information[4];
	uint8_t asc;
	uint8_t ascq;
	uint8_t field_replaceable_unit_code;
	uint8_t sense_key_specific[3];
} PACKED;

struct inquiry {
	// byte 0
	uint8_t peripheral_device_type : 5;
	uint8_t peripheral_qualifier : 3;
	// byte 1
	uint8_t : 7;                          // reserved
	bool rmb : 1;                         // removable media bit
	// byte 2
	uint8_t version;
	// byte 3
	uint8_t response_data_format : 4;
	bool hisup : 1;                       // hierarchical support
	bool normaca : 1;                     // normal aca supported
	bool : 1; bool : 1;                   // obsolete
	// byte 4
	uint8_t additional_length;
	// byte 5
	bool protect : 1;                     // protection information support
	uint8_t : 2;                          // reserved
	bool tpc : 1;                         // third party copy
	uint8_t tpgs : 2;                     // target priority group support
	bool acc : 1;                         // access controls coordinator
	bool sccs : 1;                        // SCC supported
	// byte 6
	uint8_t : 4;                          // 4 obsolete bits
	bool multip : 1;                      // multi-port
	bool vs : 1;                          // vendor-specific
	bool encserv : 1;                     // enclosure services
	bool : 1;                             // obsolete
	// byte 7
	bool vs_2 : 1;                        // vendor specific
	bool cmdque : 1;                      // command queueing support
	uint8_t : 6;                          // obsolete
	// bytes 8-15
	char vendor_id[8];
	// bytes 16-31
	char product_id[16];
	// bytes 32-35
	char product_revision[4];
} PACKED; // 36 bytes

//////////////////// STATIC VARIABLES //////////////////////

// USBD handle used by everything
static usbd_device *device;
// USB interface ID specified in the init function
// ensures we only handle control requests for the correct interface
static int interface_id;
// number of LUNs specified in the init function
static int num_luns;

// endpoints and their sizes
static struct {
	uint8_t in, in_size, out, out_size;
} endpoints;

// array specified by the user
const struct scsi_inquiry_data *inquiry_data;

// callback functions specified by the user
static uint32_t   (*num_blocks) (int lun);
static msc_result (*read_block) (int lun, uint32_t lba, uint8_t *copy_to);
static msc_result (*write_block) (int lun, uint32_t lba, const uint8_t *copy_from);
static bool       (*is_read_only) (int lun);
static bool       (*is_present) (int lun);

// saved command block structure
static struct CBW cbw;

// buffer used for rx and tx
static uint8_t buf[512];
static int buf_pos;

// state used for state management between rx / tx callbacks
static enum { STATE_EXPECTING_CBW, STATE_DATA, STATE_SEND_CSW } state;
// transfer direction, corresponds to cbw.bmCBWFlags[7]
static enum { DATA_FROM_HOST, DATA_TO_HOST } transfer_direction;

// bytes to receive or transmit
static uint32_t bytes_to_rw;
// next lba to read from or write to
static uint32_t lba;

// used for state management when sending the CSW
static bool csw_in_buffer = false;

// bytes processed, used for calculating residue
static uint32_t total_bytes_processed;
// set by process_cbw or by scsi functions
static uint8_t csw_status;
// set by process_cbw or by scsi functions
static uint8_t sense_key, sense_asc;

// manages loading the CSW into the buffer after IN receives a CLEAR_HALT request
static bool load_csw_after_unstall;
// manages loading the first LBA to be read, this needs to be done before sending data over usb
static bool initial_load_block_required = false;

/////////////////// SCSI COMMANDS /////////////////////////

// forward declaration
static void set_bad_csw(uint8_t key, uint8_t asc);
static void set_csw_based_on_result(msc_result res);

// no data transfer
static void scsi_test_unit_ready(void) {
	DEBUG_PRINT("TEST_UNIT_READY");

	if (is_present != nullptr && is_present(cbw.bCBWLUN) == false)
		set_bad_csw(SENSE_KEY_NOT_READY, SENSE_ASC_MEDIUM_NOT_PRESENT);
}

// data transfer to host
static void scsi_request_sense(void) {
	DEBUG_PRINT("REQUEST_SENSE");

	struct sense *sense = (struct sense *)buf;
	memset(sense, 0, sizeof(*sense));

	sense->valid = true;
	sense->error_code = 0x70;
	sense->sense_key = sense_key;
	sense->additional_sense_length = 10;
	sense->asc = sense_asc;
	sense->ascq = 0;

	bytes_to_rw = sizeof(*sense);
}

static void scsi_readwrite_10(void) {

	//NOTE: big-endian
	// starting LBA
	lba = (cbw.CBWCB[2]<<24) | (cbw.CBWCB[3]<<16) | (cbw.CBWCB[4]<<8) | (cbw.CBWCB[5]);
	// number of blocks
	uint16_t transfer_length = (cbw.CBWCB[7]<<8) | cbw.CBWCB[8];

	bool will_read = (cbw.CBWCB[0] == SCSI_READ_10);
	if (will_read) DEBUG_PRINT("READ(10): LBA=%08lxh, len=%u", lba, transfer_length);
	else DEBUG_PRINT("WRITE(10): LBA=%08lxh, len=%u", lba, transfer_length);

	// check for length mismatch
	if (transfer_length*512 != cbw.dCBWDataTransferLength) {
		set_bad_csw(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_INVALID_FIELD_IN_CDB);
		return;
	}

	bytes_to_rw = transfer_length*512;
}

// data transfer to host
static void scsi_inquiry(void) {

	// VPD not implemented
	bool evpd = cbw.CBWCB[1] & 0x01;
	if (evpd) {
		DEBUG_PRINT("INQUIRY(EVP) (NOT IMPLEMENTED)");

		set_bad_csw(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_INVALID_FIELD_IN_CDB);
		return;
	}

	DEBUG_PRINT("INQUIRY");

	// create an inquiry structure inside the buffer
	struct inquiry *inquiry = (struct inquiry *)buf;
	memset(buf, 0, sizeof(*inquiry));

	// INQUIRY needs to handle invalid LUNs
	bool lun_valid = (cbw.bCBWLUN < num_luns);

	// basic parts of the response
	inquiry->rmb = true;
	inquiry->version = 0x00; // don't claim compliance with anything
	inquiry->response_data_format = 2;
	inquiry->additional_length = sizeof(*inquiry) - 4;

	if (!lun_valid) {
		// LUN not supported
		inquiry->peripheral_qualifier = INQUIRY_PERIPHERAL_QUALIFIER_NOT_SUPPORTED;
	} else if (inquiry_data != nullptr) {
		memcpy(inquiry->vendor_id, inquiry_data->vendor_id, sizeof(inquiry->vendor_id));
		memcpy(inquiry->product_id, inquiry_data->product_id, sizeof(inquiry->product_id));
		memcpy(inquiry->product_revision, inquiry_data->product_revision, sizeof(inquiry->product_revision));
	} else /* inquiry_data == nullptr */ {
		memcpy(inquiry->vendor_id, "Generic ", sizeof(inquiry->vendor_id));
		memcpy(inquiry->product_id, "MS Device LUN_XX", sizeof(inquiry->product_id));
		memcpy(inquiry->product_revision, "1.00", sizeof(inquiry->product_revision));
		char tens = (cbw.bCBWLUN > 9) ? '1' : '0';
		char ones = '0' + ((cbw.bCBWLUN > 9) ? (cbw.bCBWLUN-10) : cbw.bCBWLUN);

		inquiry->product_id[14] = tens;
		inquiry->product_id[15] = ones;
	}

	bytes_to_rw = sizeof(*inquiry);
}

// no data transfer
static void scsi_send_diagnostic(void) {
	DEBUG_PRINT("SEND_DIAGNOSTIC");
	// don't need to do anything
}


// data transfer to host
static void scsi_read_capacity_10(void) {
	DEBUG_PRINT("READ_CAPACITY(10)");

	// last addressable lba in the device
	uint32_t max_lba = num_blocks(cbw.bCBWLUN) - 1;

	uint8_t *lba_bytes = (uint8_t *) &max_lba;

	// send result as big-endian

	// highest-numbered lba
	buf[0] = lba_bytes[3];
	buf[1] = lba_bytes[2];
	buf[2] = lba_bytes[1];
	buf[3] = lba_bytes[0];

	// logical block size
	buf[4] = 0;
	buf[5] = 0;
	buf[6] = 2; // 512 
	buf[7] = 0;

	bytes_to_rw = 8;
}

// no data transfer
static void scsi_prevent_allow_medium_removal(void) {
	bool prevent = cbw.CBWCB[4] & 0x01;
	DEBUG_PRINT("PREVENT_ALLOW_MEDIUM_REMOVAL (%s)", prevent ? "PREVENT" : "ALLOW");
}

// data transfer to host
static void scsi_mode_sense_6(void) {
	uint8_t page_code = cbw.CBWCB[2] & 0b00111111;
	DEBUG_PRINT("MODE_SENSE(6) - PAGE(%02xh)", page_code);

	bool read_only = (write_block == nullptr) || (is_read_only != nullptr && is_read_only(cbw.bCBWLUN));

	buf[0] = 3; // number of bytes that follow
	buf[1] = 0; // medium type = SBC
	buf[2] = read_only ? 0x80 : 0x00; // device specific parameters, bit7 = read-only
	buf[3] = 0; // no block descriptors to follow

	bytes_to_rw = 4;
}

static void scsi_readwrite_6(void) {
	lba = (cbw.CBWCB[1]<<16) | (cbw.CBWCB[2]<<8) | (cbw.CBWCB[3]);
	uint16_t transfer_length = cbw.CBWCB[4];

	if (transfer_length == 0) transfer_length = 256; // as per standard

	bool will_read = (cbw.CBWCB[0] == SCSI_READ_6);
	if (will_read) DEBUG_PRINT("READ(10): LBA=%08lxh, len=%u", lba, transfer_length);
	else DEBUG_PRINT("WRITE(10): LBA=%08lxh, len=%u", lba, transfer_length);

	// check for length mismatch
	if (transfer_length*512 != cbw.dCBWDataTransferLength) {
		set_bad_csw(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_INVALID_FIELD_IN_CDB);
		return;
	}

	bytes_to_rw = transfer_length*512;
}

static void scsi_unsupported_command(void) {
	DEBUG_PRINT("UNSUPPORTED_COMMAND(%02xh)", cbw.CBWCB[0]);

	set_bad_csw(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_UNSUPPORTED_COMMAND);
}


////////////////////// FUNCTIONS ///////////////////////////

// prepares buffer to receive data from wherever
static void reset_buffer(void) {
	buf_pos = 0;
	bytes_to_rw = 0;
	csw_in_buffer = false;
}

static void stall_bulk_out(void) {
	DEBUG_PRINT(" <<STALL OUT>>");
	usbd_ep_stall_set(device, endpoints.out, true);
}

static void stall_bulk_in(void) {
	DEBUG_PRINT(" <<STALL IN>>");
	usbd_ep_stall_set(device, endpoints.in, true);
}

// forward declaration
static void transfer_csw(void);

// appropriately stalls endpoints and prepares for csw transfer
static void command_end_early(void) {
	state = STATE_SEND_CSW;
	if (cbw.dCBWDataTransferLength == 0) {
		transfer_csw();
	} else if (transfer_direction == DATA_FROM_HOST) {
		stall_bulk_out();
		transfer_csw();
	} else /* transfer_direction == DATA_TO_HOST */ {
		stall_bulk_in();
		load_csw_after_unstall = true;
	}
}

// helper function
static void set_bad_csw(uint8_t key, uint8_t asc) {
	csw_status = CSW_STATUS_CHECK_CONDITION;
	sense_key = key;
	sense_asc = asc;
}

// forward declaration
static void transfer_data(void);

// calls transfer_data to do the actual transfer
static void transfer_csw(void) {
	// load the CSW into the buffer if it isn't already
	if (!csw_in_buffer) {
		struct CSW *csw = (struct CSW *)buf;
		csw->dCSWSignature = CSW_SIGNATURE;
		csw->dCSWTag = cbw.dCBWTag;
		csw->dCSWDataResidue = cbw.dCBWDataTransferLength - total_bytes_processed;
		csw->bCSWStatus = csw_status;

		bytes_to_rw = sizeof(struct CSW);
		buf_pos = 0;
		transfer_direction = DATA_TO_HOST;
		csw_in_buffer = true;

		if (csw_status == CSW_STATUS_OK) DEBUG_PRINT(" --> OK\r\n");
		else DEBUG_PRINT(" --> CHECK_CONDITION SENSE=%02xh ASC=%02xh\r\n", sense_key, sense_asc);
	}

	// reuse the transfer_data function to perform the transfer
	transfer_data();

	if (bytes_to_rw == 0) {
		// we are done, ready for the next command
		state = STATE_EXPECTING_CBW;
		reset_buffer();
	}
}

// processes a CBW after it has been fully received
static void process_cbw(void) {
	// we need to save the CBW for later anyway, so memcpy it to a saved buffer
	memcpy(&cbw, buf, sizeof(cbw));

	// check cbw signature
	if (cbw.dCBWSignature != CBW_SIGNATURE) {
		DEBUG_PRINT("UNEXPECTED CBW SIGNATURE, GOT (0x%08lx)\r\n", cbw.dCBWSignature);
		reset_buffer();
		state = STATE_EXPECTING_CBW;
		// just wait for a proper cbw then?
		return;
	}

	// reset stuff
	reset_buffer();
	total_bytes_processed = 0;
	csw_status = CSW_STATUS_OK;
	load_csw_after_unstall = false;

	// reset sense, unless reading sense data
	if (cbw.CBWCB[0] != SCSI_REQUEST_SENSE) {
		sense_key = SENSE_KEY_NO_SENSE;
		sense_asc = SENSE_ASC_NO_ADDITIONAL_INFO;
	}

	DEBUG_PRINT("LUN(%u) LEN= %-8lu - ", cbw.bCBWLUN, cbw.dCBWDataTransferLength);

	// transfer direction is specified in the cbw
	if (cbw.bmCBWFlags & 0x80) transfer_direction = DATA_TO_HOST;
	else transfer_direction = DATA_FROM_HOST;

	// check LUN
	if (cbw.bCBWLUN >= num_luns && cbw.CBWCB[0] != SCSI_INQUIRY) {
		set_bad_csw(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_LUN_NOT_SUPPORTED);
		command_end_early();
		return;
	}

	switch (cbw.CBWCB[0]) {
		case SCSI_TEST_UNIT_READY:
			scsi_test_unit_ready();
			break;
		case SCSI_REQUEST_SENSE:
			scsi_request_sense();
			break;
		case SCSI_INQUIRY:
			scsi_inquiry();
			break;
		case SCSI_MODE_SENSE_6:
			scsi_mode_sense_6();
			break;
		case SCSI_SEND_DIAGNOSTIC:
			scsi_send_diagnostic();
			break;
		case SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL:
			scsi_prevent_allow_medium_removal();
			break;
		case SCSI_READ_CAPACITY_10:
			scsi_read_capacity_10();
			break;
		case SCSI_READ_10:
		case SCSI_WRITE_10:
			scsi_readwrite_10();
			break;
		case SCSI_READ_6:
		case SCSI_WRITE_6:
			scsi_readwrite_6();
			break;
		default:
			scsi_unsupported_command();
	}

	// limit r/w bytes to that requested in the cbw
	bytes_to_rw = MIN(bytes_to_rw, cbw.dCBWDataTransferLength);

	state = STATE_DATA;

	//NOTE: transfer_data callback handles when we need to stall the endpoint
	//      in the case where the host wants more data than we provide.
	//      Also handles cases where the data transfer is zero.
	transfer_data();
}

// helper function for when calling user provided callbacks
static void set_csw_based_on_result(msc_result res) {
	if (res == MSC_RESULT_OK) return;

	csw_status = CSW_STATUS_CHECK_CONDITION;
	switch (res) {
		case MSC_RESULT_READ_ONLY:
			sense_key = SENSE_KEY_DATA_PROTECT;
			sense_asc = SENSE_ASC_WRITE_PROTECTED;
			break;
		case MSC_RESULT_IO_ERROR:
			sense_key = SENSE_KEY_MEDIUM_ERROR;
			sense_asc = (transfer_direction == DATA_TO_HOST) ? SENSE_ASC_UNRECOVERED_READ_ERROR : SENSE_ASC_WRITE_FAULT;
			break;
		case MSC_RESULT_INVALID_PARAMETER:
			sense_key = SENSE_KEY_ILLEGAL_REQUEST;
			sense_asc = SENSE_ASC_LBA_OUT_OF_RANGE;
			break;
		default:
			break;
	}
}

// transfers data from or to the host
static void transfer_data(void) {

	// load first lba into buffer if required for IN (ie. READ(10))
	bool command_is_read = (cbw.CBWCB[0] == SCSI_READ_10) || (cbw.CBWCB[0] == SCSI_READ_6);
	if (state == STATE_DATA && command_is_read && bytes_to_rw > 0 && total_bytes_processed == 0) {
		msc_result res = read_block(cbw.bCBWLUN, lba, buf);

		if (res != MSC_RESULT_OK) {
			set_csw_based_on_result(res);
			command_end_early();
			return;
		}
		lba++;
		buf_pos = 0;
	}

	// handle cases where Hi/Ho > Di/Do
	if (bytes_to_rw == 0 && state == STATE_DATA && total_bytes_processed < cbw.dCBWDataTransferLength) {
		command_end_early();
		return;
	}

	// main data transfer
	if (bytes_to_rw > 0) {
		int bytes_left_to_transfer = MIN(bytes_to_rw, sizeof(buf) - buf_pos);
		int bytes_transfered = 
			(transfer_direction == DATA_TO_HOST) ?
			usbd_ep_write_packet(device, endpoints.in, &buf[buf_pos], MIN(endpoints.in_size, bytes_left_to_transfer)) :
			usbd_ep_read_packet(device, endpoints.out, &buf[buf_pos], MIN(endpoints.out_size, bytes_left_to_transfer));
		total_bytes_processed += bytes_transfered;
		buf_pos += bytes_transfered;
		bytes_to_rw -= bytes_transfered;
	}

	// if we reached the end of the buffer, surely we are transferring data to/from backing storage
	if (state == STATE_DATA && buf_pos == sizeof(buf)) {
		msc_result res;
		if (transfer_direction == DATA_TO_HOST) {
			res = read_block(cbw.bCBWLUN, lba, buf);
		} else {
			// return read only when the write callback is nullptr
			res = (write_block != nullptr) ? write_block(cbw.bCBWLUN, lba, buf) : MSC_RESULT_READ_ONLY;
		}

		if (res != MSC_RESULT_OK) {
			// don't report processing data if we didn't write it to backing storage
			if(transfer_direction == DATA_FROM_HOST) total_bytes_processed -= sizeof(buf);
			set_csw_based_on_result(res);
			command_end_early();
			return;
		}
		// increment lba and reset buf
		lba++;
		buf_pos = 0;
	}

	// handle Hi/Ho == Di/Do, including Hi/Ho == 0
	if (bytes_to_rw == 0 && state == STATE_DATA && total_bytes_processed == cbw.dCBWDataTransferLength) {
		state = STATE_SEND_CSW;
		if (transfer_direction == DATA_FROM_HOST || cbw.dCBWDataTransferLength == 0) transfer_csw();
	}
}

// receives a CBW until 31 bytes have been received
static void receive_cbw(void) {
	// remaining cbw we need to read
	int cbw_remaining = 31 - buf_pos;


	// receive as much as we can from the endpoint
	int bytes_to_receive = MIN(cbw_remaining, endpoints.out_size);

	int bytes_received = usbd_ep_read_packet(device, endpoints.out, &buf[buf_pos], bytes_to_receive);
	buf_pos += bytes_received;

	if (buf_pos < 31) return; // still need more data

	process_cbw();
}

// receives a usb packet, calls receive_cbw or transfer_data(rx) as required
static void endpoint_rx_ready_callback(usbd_device *usbd_dev, uint8_t ep) {
	(void)usbd_dev;
	(void)ep;

	if (state == STATE_EXPECTING_CBW) receive_cbw();
	else if (state == STATE_DATA) transfer_data();
	// csw not transferred in this callback
}

// transmits another packet or a csw as required
static void endpoint_tx_completed_callback(usbd_device *usbd_dev, uint8_t ep) {
	(void)usbd_dev;
	(void)ep;

	// cbw not transferred in this callback
	if (state == STATE_DATA) transfer_data();
	else if (state == STATE_SEND_CSW) transfer_csw();
}

// hook this to load the csw when we clear the stall condition after failing a transfer
static enum usbd_request_return_codes control_callback_endpoint(
	struct usb_setup_data *req, 
	uint8_t **control_buf, uint16_t *len
) {
	(void)control_buf;
	(void)len;

	// the host shall attempt to receive a csw after clearing the stall condition
	if (req->bRequest == USB_REQ_CLEAR_FEATURE && req->wValue == USB_FEAT_ENDPOINT_HALT && req->wIndex == endpoints.in) {
		usbd_ep_stall_set(device, endpoints.in, false);
		if (load_csw_after_unstall) {
			state = STATE_SEND_CSW;
			transfer_csw();
		}
		return USBD_REQ_HANDLED;
	} else return USBD_REQ_NEXT_CALLBACK;
}

static enum usbd_request_return_codes control_callback_interface(
	struct usb_setup_data *req,
	uint8_t **control_buf, uint16_t *len
) {
	// only respond to the mass storage interface
	if (req->wIndex != interface_id) return USBD_REQ_NEXT_CALLBACK;

	if (req->bRequest == 0xFF) { /* Bulk-Only Mass Storage Reset */
		DEBUG_PRINT("\r\n\r\n<<<BULK-ONLY MASS STORAGE RESET>>>\r\n\r\n");
		state = STATE_EXPECTING_CBW;
		reset_buffer();
		load_csw_after_unstall = false;
		initial_load_block_required = false;
		return USBD_REQ_HANDLED;
	} else if (req->bRequest == 0xFE) { /* Get Max LUN */
		// return the highest numbered lun
		*control_buf[0] = num_luns - 1;
		*len = 1;
		return USBD_REQ_HANDLED;
	}

	// default case
	return USBD_REQ_NOTSUPP;
}

// usbd control callback, used for MSC requests and endpoints requests for endpoints.in
static enum usbd_request_return_codes control_callback(
	usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **control_buf, uint16_t *len,
	usbd_control_complete_callback *complete
) {
	(void)usbd_dev;
	(void)complete;

	if ((req->bmRequestType & (USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT)) == (USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE)) {
		return control_callback_interface(req, control_buf, len);
	} else if ((req->bmRequestType & (USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT)) == (USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_ENDPOINT)) {
		return control_callback_endpoint(req, control_buf, len);
	} else return USBD_REQ_NEXT_CALLBACK;
}

static void set_config_callback(usbd_device *usbd_dev, uint16_t wValue) {
	(void)usbd_dev;
	(void)wValue;

	usbd_ep_setup(device, endpoints.in, USB_ENDPOINT_ATTR_BULK, endpoints.in_size, endpoint_tx_completed_callback);
	usbd_ep_setup(device, endpoints.out, USB_ENDPOINT_ATTR_BULK, endpoints.out_size, endpoint_rx_ready_callback);
	usbd_register_control_callback(
		device, 
		0, // respond to all requests, as we need to handle multiple types
		0,
		control_callback
	);
}

void msc_init_multi(
    usbd_device *_device,
    uint8_t _interface_id,
    uint8_t ep_in, uint8_t ep_in_size,
    uint8_t ep_out, uint8_t ep_out_size,
    int _num_luns,
    const struct scsi_inquiry_data *_inquiry_data,
    uint32_t   (*_num_blocks) (int lun),
    msc_result (*_read_block) (int lun, uint32_t lba, uint8_t *copy_to),
    msc_result (*_write_block) (int lun, uint32_t lba, const uint8_t *copy_from),
    bool       (*_is_read_only) (int lun),
    bool       (*_is_present) (int lun)
) {
	device = _device;
	interface_id = _interface_id;
	endpoints.in = ep_in;
	endpoints.in_size = ep_in_size;
	endpoints.out = ep_out;
	endpoints.out_size = ep_out_size;
	num_luns = _num_luns;
	inquiry_data = _inquiry_data;
	num_blocks = _num_blocks;
	read_block = _read_block;
	write_block = _write_block;
	is_read_only = _is_read_only;
	is_present = _is_present;

	SETUP_DEBUG_PRINT();

	usbd_register_set_config_callback(device, set_config_callback);
}