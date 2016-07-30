#ifndef _XBEE_H_
#define _XBEE_H_

#include <stddef.h>
#include <stdint.h>

#ifndef SPECIAL_SECTION
#define SPECIAL_SECTION
#endif

#ifndef XBEE_GUARD_TIME
#define XBEE_GUARD_TIME 2
#endif /* XBEE_GUARD_TIME */

typedef int (*xbee_write_fun_t)(void * ptr, const void *buf, size_t nbyte);
typedef int (*xbee_read_fun_t)(void * ptr, void *buf, size_t nbyte);
typedef unsigned (*xbee_sleep_t)(unsigned sec);

/*! xbee_uart_interface_t provides an abstration to a uart interface
 *
 * xbee_uart_interface_t assumes hardware flow control is supported and is active 
 * xbee_uart_interface_t assumes nonblocking reads and writes
 * xbee_uart_interface_t assumes that the UART and XBee initially share the same baud rate
 *
 * */
typedef struct {
    void * ptr;

    xbee_write_fun_t write;             /*! Write bytes to UART, conform to posix write interface */
    xbee_read_fun_t read;               /*! Read bytes from UART, conform to posix read interface */
    xbee_sleep_t sleep;
} xbee_uart_interface_t;

typedef struct {
    xbee_uart_interface_t * uart;

    size_t recv_max_size;
    uint8_t * recv;
    size_t recv_idx;
    size_t recv_size;
} xbee_interface_t;


#define XBEE_REC_BUF_SIZE (234)
#define XBEE_MAX_FRAME_SIZE (117)

/*! Opens XBee via provided uart interface
 *
 * \param[out] xbee New structure to xbee
 * \param[in] uart Pointer to initialized UART interface 
 * \param recv_buffer_size Size of receive buffer.  
 *      Recommend buffer of more than 234 bytes (XBEE_REC_BUF_SIZE)
 *
 * \return 0 for success, X otherwise
 */
int xbee_open(xbee_interface_t * xbee, xbee_uart_interface_t * uart, size_t recv_buffer_size, void * recv_buffer) SPECIAL_SECTION;

/*! Sends frame to XBee
 *
 * \param[in] xbee Pointer to initialized XBee interface
 * \param[in] frame_size Size of frame to send
 * \param[in] frame_data Frame data to send
 *
 * \return 0 indicates frame was successfully written, otherwise returns error code from xbee_write_fun_t
 *
 */
int xbee_send_frame(xbee_interface_t * xbee, size_t frame_size, const void * frame_data) SPECIAL_SECTION;

/*! Receives frame from XBee
 *
 * \param[in] xbee Pointer to initialized XBee interface
 * \param[in] frame_out_size Size of frame_out buffer
 * \param[inout] frame_out Pointer to buffer of frame_out_size
 *
 * \return 0 indicates partial frame was read or no data was recieved
 *        >0 indicates frame was read and was output in frame_out
 *        <0 indicates error reading data, and returns error code from xbee_read_fun_t
 */
int xbee_recv_frame(xbee_interface_t * xbee, size_t frame_out_size, void * frame_out) SPECIAL_SECTION;

typedef enum {
    /* Commands */
    XBEE_TRANSMIT = 0x00,
    XBEE_TRANSMIT_16_BIT = 0x01,
    XBEE_AT_COMMAND = 0x08,
    XBEE_AT_QUEUE_PARAMETER = 0x09,
    XBEE_REMOTE_AT_COMMAND = 0x17,

    /* Responses */
    XBEE_RECEIVE = 0x80,
    XBEE_AT_RESPONSE = 0x88,
    XBEE_TRANSMIT_STATUS = 0x89,
    XBEE_MODEM_STATUS = 0x8A,
    XBEE_RECEIVE_16_BIT = 0x81,
    XBEE_REMOTE_AT_RESPONSE = 0x97,
} xbee_api_id_t;

typedef enum {
    XBEE_16_BIT,
    XBEE_64_BIT,
    XBEE_16_BIT_BROADCAST,
    XBEE_64_BIT_BROADCAST,
} xbee_address_type_t;

typedef struct {
    xbee_address_type_t type;

    union {
        uint64_t address;
        uint16_t network_address;
    } addr;
} xbee_address_t;

/*! Functions to send AT commands
 * \param frame_id Frame id in transmit response, 0 to disable response
 */
int xbee_at_command(xbee_interface_t * xbee, 
        uint8_t frame_id, char * at_command, size_t param_size, const void * param) SPECIAL_SECTION;
int xbee_at_queue_parameter(xbee_interface_t * xbee, 
        uint8_t frame_id, char * at_command, size_t param_size, const void * param) SPECIAL_SECTION;
int xbee_remote_at_command(xbee_interface_t * xbee, 
        const xbee_address_t * address, uint8_t options,
        uint8_t frame_id, char * at_command, size_t param_size, const void * param) SPECIAL_SECTION;

#define XBEE_DISABLE_ACK        (0x01)
#define XBEE_BROADCAST_PAN_ID   (0x04)

/*! Transmit packet to specified address 
 *
 * \param frame_id Frame id in transmit response, 0 to disable response
 * \param option Either XBEE_DISABLE_ACK or XBEE_BROADCAST_PAN_ID
 * */
int xbee_transmit(xbee_interface_t * xbee, uint8_t frame_id, const xbee_address_t * address, uint8_t option, size_t data_size, const void * data) SPECIAL_SECTION;

typedef struct {
    xbee_api_id_t api_id;
    uint8_t frame_id; /* XBEE_MODEM_STATUS, XBEE_TRANSMIT_STATUS, XBEE_AT_RESPONSE, XBEE_REMOTE_AT_RESPONSE */
        
    union {
        uint8_t status; /*! XBEE_MODEM_STATUS, XBEE_TRANSMIT_STATUS */
        struct {
            char at_command[3];
            uint8_t status;
            uint64_t responder_address;         /*! Only XBEE_REMOTE_AT_RESPONSE */
            uint16_t responder_network_address; /*! Only XBEE_REMOTE_AT_RESPONSE */
            size_t data_size;
            const uint8_t * data;
        } at_command_response; /*! XBEE_AT_RESPONSE, XBEE_REMOTE_AT_RESPONSE */
        struct {
            uint64_t responder_address;         /*! Only XBEE_RECEIVE */
            uint16_t responder_network_address; /*! Only XBEE_RECEIVE_16_BIT */
            int8_t rssi;
            uint8_t options;
            size_t packet_size;
            const void * packet_data;
        } receive; /*! XBEE_RECEIVE, XBEE_RECEIVE_16_BIT */
    } frame;
} xbee_parsed_frame_t;

#define XBEE_UNKNOWN_API_ID       (-1)
#define XBEE_WRONG_LENGTH_FOR_API (-2)

/*! Parses received XBee frame.  
 *
 * Only parses frames that come from a XBee.
 *
 * For XBEE_AT_RESPONSE, XBEE_REMOTE_AT_RESPONSE, XBEE_RECEIVE and XBEE_RECEIVE_16_BIT,
 * data in pointer frame is used in parsed_frame.
 *
 * \return Returns 0 if api_id is recongnized and expect length matches.
 *         Returns XBEE_UNKNOWN_API_ID or XBEE_WRONG_LENGTH_FOR_API otherwise
 */
int xbee_parse_frame(xbee_parsed_frame_t * parsed_frame, size_t frame_size, const void * frame) SPECIAL_SECTION;


#endif /* _XBEE_H_ */
