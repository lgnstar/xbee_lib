#include "xbee.h"
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#define write(buf, nbyte) xbee->uart->write(xbee->uart->ptr, buf, nbyte)
#define read(buf, nbyte) xbee->uart->read(xbee->uart->ptr, buf, nbyte)

static int xbee_init(xbee_interface_t * xbee)
{
    char c = '+';
    xbee->uart->sleep(XBEE_GUARD_TIME);
    int ret;
    for(int i = 0; i < 3; ++i)
    {
        ret = write(&c, 1);
        if(ret != 1)
        {
            return ret;
        }
    }
    xbee->uart->sleep(XBEE_GUARD_TIME);

    char api_seq[] = "ATAP 0\r";
    ret = write(api_seq, sizeof(api_seq)-1);
    if(ret != sizeof(api_seq)-1)
    {
        return ret;
    }

    return 0;
}

int xbee_open(xbee_interface_t * xbee, xbee_uart_interface_t * uart, 
        size_t recv_buffer_size, void * recv_buffer)
{
    assert(xbee);
    assert(uart);
    assert(recv_buffer);

    memset(xbee, 0, sizeof(xbee));

    xbee->uart = uart;
    xbee->recv_max_size = recv_buffer_size;
    xbee->recv = recv_buffer;
    xbee->recv_idx = 0;
    xbee->recv_size = 0;

    return xbee_init(xbee);
}

#define XBEE_FRAME_DELIM 0x7E
#define XBEE_FRAME_ESCAPE 0x7D
#define XBEE_XON 0x11
#define XBEE_XOFF 0x13

/*! Writes bytes with escaping */
static int xbee_write_bytes(xbee_interface_t * xbee, 
        size_t nbytes, const void * buf, uint8_t *accum)
{
    assert(xbee);
    assert(buf);
    assert(accum);

    const uint8_t * bytes = buf;
    uint8_t escape_buf[2];
    escape_buf[0] = XBEE_FRAME_ESCAPE;

    int ret;
    size_t off = 0;
    for(size_t i = 0; i < nbytes; ++i)
    {
        *accum += bytes[i];

        if(bytes[i] == XBEE_FRAME_DELIM || 
           bytes[i] == XBEE_FRAME_ESCAPE || 
           bytes[i] == XBEE_XON || bytes[i] == XBEE_XOFF)
        {
            /* Write everything up to the escaped byte */
            size_t to_write = i-off;
            ret = write(buf+off, to_write);
            if(ret != to_write)
            {
                return ret;
            }

            escape_buf[1] = bytes[i] ^ 0x20;

            ret = write(escape_buf, sizeof(escape_buf));
            if(ret != sizeof(escape_buf))
            {
                return ret;
            }

            off = i+1;
        }
    }

    /* Write remaining bytes */
    size_t to_write = nbytes-off;
    ret = write(buf+off, to_write);
    if(ret != to_write)
    {
        return ret;
    }

    return 0;
}

static int xbee_start_frame(xbee_interface_t * xbee, 
        uint16_t total_frame_length, uint8_t * accum)
{
    assert(xbee);
    assert(accum);

    char c = XBEE_FRAME_DELIM;
    int ret = write(&c, 1);
    if(ret != 1)
    {
        return ret;
    }

    *accum = 0;

    uint8_t buf[2];
    buf[0] = total_frame_length >> 8;
    buf[1] = total_frame_length & 0xFF;

    ret = xbee_write_bytes(xbee, sizeof(buf), buf, accum);
    *accum = 0;
    return ret;
}

static int xbee_finish_frame(xbee_interface_t * xbee, uint8_t accum)
{
    uint8_t dummy = 0;
    accum = 0xFF - accum;
    return xbee_write_bytes(xbee, 1, &accum, &dummy);
}

int xbee_send_frame(xbee_interface_t * xbee, 
        size_t frame_size, const void * frame_data)
{
    uint8_t accum;
    int ret = xbee_start_frame(xbee, frame_size, &accum);
    if(ret != 0)
    {
        return ret;
    }

    ret = xbee_write_bytes(xbee, frame_size, frame_data, &accum);
    if(ret != 0)
    {
        return ret;
    }

    return xbee_finish_frame(xbee, accum);
}

static inline void xbee_drop_byte(xbee_interface_t * xbee)
{
    xbee->recv_idx += 1;
    if(xbee->recv_idx > xbee->recv_max_size)
    {
        xbee->recv_idx = 0;
    }
    xbee->recv_size -= 1;
}

static inline uint8_t xbee_get_byte(xbee_interface_t * xbee, size_t i)
{
    assert(xbee);
    assert(i < xbee->recv_size);
    assert(xbee->recv_idx < xbee->recv_max_size);
    
    size_t idx = i + xbee->recv_idx;
    if(idx > xbee->recv_max_size)
    {
        idx -= xbee->recv_max_size;
    }

    return xbee->recv[idx];
}

#define XBEE_ERR_FOUND_START (-1)
#define XBEE_NOT_ENOUGH_DATA (-2)

static int xbee_get_next_byte(xbee_interface_t * xbee, 
        size_t *idx, uint8_t * byte_out)
{
    if(*idx >= xbee->recv_size)
    {
        return XBEE_NOT_ENOUGH_DATA;
    }

    *byte_out = xbee_get_byte(xbee, *idx);

    if(*byte_out == XBEE_FRAME_DELIM)
    {
        return XBEE_ERR_FOUND_START;
    }

    if(*byte_out == XBEE_FRAME_ESCAPE)
    {
        if(*idx+1 >= xbee->recv_size)
        {
            return XBEE_NOT_ENOUGH_DATA;
        }

        *byte_out = xbee_get_byte(xbee, *idx+1);
        if(*byte_out == XBEE_FRAME_DELIM)
        {
            return XBEE_ERR_FOUND_START;
        }

        *byte_out ^= 0x20;
        *idx += 2;
        return 0;
    }
    else
    {
        *idx += 1;
        return 0;
    }
}

static int xbee_decode_frame(xbee_interface_t * xbee, 
        size_t frame_out_size, void * frame_out)
{
    assert(xbee->recv_size <= xbee->recv_max_size);

    uint8_t *bytes_out = frame_out;

    /* Need at least 6 bytes for a frame 
     * 1 delim
     * 2 length
     * 1 api ID
     * 1+ data bytes
     * 1 checksum
     *
     * Using 6 also meanss xbee_get_next_byte for len1 and len2 will never fail
     * due to lack of data, only other errors.
     * */
    while(xbee->recv_size >= 6)
    {
        if(xbee_get_byte(xbee, 0) != XBEE_FRAME_DELIM)
        {
            xbee_drop_byte(xbee);
            continue;
        }

        size_t idx = 1;
        uint8_t len1, len2;
        int ret = xbee_get_next_byte(xbee, &idx, &len1);
        if(ret != 0)
        {
            xbee_drop_byte(xbee);
            continue;
        }

        ret = xbee_get_next_byte(xbee, &idx, &len2);
        if(ret != 0)
        {
            xbee_drop_byte(xbee);
            continue;
        }

        /* Buffer starts with XBEE_FRAME_DELIM */
        uint16_t length = len1 << 8 | len2;

        /* 1 for start delim, 2 for length, 1 for checksum */
        size_t required_bytes = length + 4;  
        if(required_bytes > xbee->recv_max_size || length+1 > frame_out_size)
        {
            /* FIXME: Handle overflow better */
            xbee_drop_byte(xbee);
            continue;
        }

        uint8_t accum = 0;
        for(size_t i = 0; i < length+1; ++i)
        {
            ret = xbee_get_next_byte(xbee, &idx, &bytes_out[i]);
            if(ret == XBEE_NOT_ENOUGH_DATA)
            {
                if(xbee->recv_size == xbee->recv_max_size)
                {
                    /* FIXME: Handle overflow better */
                    xbee_drop_byte(xbee);
                    goto next;
                }
                else
                {
                    return 0;
                }
            }
            else if(ret != 0)
            {
                xbee_drop_byte(xbee);
                goto next;
            }

            accum += bytes_out[i];
        }

        if(accum == 0xFF)
        {
            return length;
        }
        else
        {
            xbee_drop_byte(xbee);
            continue;
        }
next:
        continue;
    }

    return 0;
}

static int xbee_fill_buffer(xbee_interface_t * xbee)
{
    assert(xbee->recv_size < xbee->recv_max_size);

    size_t read_start = xbee->recv_idx + xbee->recv_size;
    size_t read_end;
    if(read_start < xbee->recv_max_size)
    {
        read_end = xbee->recv_max_size;
    }
    else
    {
        read_start -= xbee->recv_max_size;
        read_end = xbee->recv_idx;
    }

    size_t read_len = read_end - read_start;
    int ret = read(xbee->recv+read_start, read_len);
    if(ret > 0)
    {
        xbee->recv_size += ret;
    }

    if(ret == read_len &&  /* First read complete */
       read_end != xbee->recv_idx && /* Buffer is not full */
       xbee->recv_size < xbee->recv_max_size /* Buffer is not full */ )
    {
        size_t read = ret;

        read_start = 0; 
        read_end = xbee->recv_idx;
        read_len = read_end - read_start;
        ret = read(xbee->recv+read_start, read_len);

        if(ret > 0)
        {
            xbee->recv_size += ret;
            read += ret;
        }

        return read;
    }
    else
    {
        return ret;
    }
}

int xbee_recv_frame(xbee_interface_t * xbee, 
        size_t frame_out_size, void * frame_out)
{
    int ret = xbee_decode_frame(xbee, frame_out_size, frame_out);

    /* Already have a frame, return it */
    if(ret > 0)
    {
        return ret;
    }

    ret = xbee_fill_buffer(xbee);
    if(ret < 0)
    { 
        return ret;
    }

    return xbee_decode_frame(xbee, frame_out_size, frame_out);
}

int xbee_at_command(xbee_interface_t * xbee, 
        uint8_t frame_id, char * at_command, 
        size_t param_size, const void * param)
{
    uint8_t accum;
    int ret = xbee_start_frame(xbee, 4+param_size, &accum);
    if(ret != 0)
    {
        return ret;
    }

    uint8_t buf[4];
    buf[0] = XBEE_AT_COMMAND;
    buf[1] = frame_id;
    buf[2] = at_command[0];
    buf[3] = at_command[1];

    ret = xbee_write_bytes(xbee, sizeof(buf), buf, &accum);
    if(ret != 0)
    {
        return ret;
    }

    ret = xbee_write_bytes(xbee, param_size, param, &accum);
    if(ret != 0)
    {
        return ret;
    }

    return xbee_finish_frame(xbee, accum);
}

int xbee_at_queue_parameter(xbee_interface_t * xbee, 
        uint8_t frame_id, char * at_command, 
        size_t param_size, const void * param)
{
    uint8_t accum;
    int ret = xbee_start_frame(xbee, 4+param_size, &accum);
    if(ret != 0)
    {
        return ret;
    }

    uint8_t buf[4];
    buf[0] = XBEE_AT_QUEUE_PARAMETER;
    buf[1] = frame_id;
    buf[2] = at_command[0];
    buf[3] = at_command[1];

    ret = xbee_write_bytes(xbee, sizeof(buf), buf, &accum);
    if(ret != 0)
    {
        return ret;
    }

    ret = xbee_write_bytes(xbee, param_size, param, &accum);
    if(ret != 0)
    {
        return ret;
    }

    return xbee_finish_frame(xbee, accum);
}

int xbee_remote_at_command(xbee_interface_t * xbee, 
        const xbee_address_t * address, uint8_t options,
        uint8_t frame_id, char * at_command, 
        size_t param_size, const void * param)
{
    uint8_t accum;
    int ret = xbee_start_frame(xbee, 15+param_size, &accum);
    if(ret != 0)
    {
        return ret;
    }

    uint8_t buf[15];
    buf[0] = XBEE_REMOTE_AT_COMMAND;
    buf[5-4] = frame_id;

    if(address->type == XBEE_64_BIT)
    {
        for(size_t i = 0; i < 8; ++i)
        {
            buf[6+i-4] = (address->addr.address >> (64 - 8*(i+1))) & 0xFF;
        }
        buf[14-4] = 0xFF;
        buf[15-4] = 0xFE;
    }
    else if(address->type == XBEE_16_BIT)
    {
        buf[14-4] = address->addr.network_address >> 8;
        buf[15-4] = address->addr.network_address & 0xFF;
    }
    else
    {
        assert(address->type == XBEE_64_BIT_BROADCAST);

        for(size_t i = 0; i < 6; ++i)
        {
            buf[6+i-4] = 0;
        }
        buf[12-4] = 0xFF;
        buf[13-4] = 0xFF;
        buf[14-4] = 0xFF;
        buf[15-4] = 0xFE;
    }

    buf[16-4] = options;
    buf[17-4] = at_command[0];
    buf[18-4] = at_command[1];

    ret = xbee_write_bytes(xbee, sizeof(buf), buf, &accum);
    if(ret != 0)
    {
        return ret;
    }

    ret = xbee_write_bytes(xbee, param_size, param, &accum);
    if(ret != 0)
    {
        return ret;
    }

    return xbee_finish_frame(xbee, accum);
}

int xbee_transmit(xbee_interface_t * xbee, uint8_t frame_id, 
        const xbee_address_t * address, uint8_t option, 
        size_t data_size, const void * data)
{
    uint8_t accum;
    if(address->type == XBEE_16_BIT || address->type == XBEE_16_BIT_BROADCAST)
    {
        uint8_t buf[5];
        buf[0] = XBEE_TRANSMIT_16_BIT;
        buf[1] = frame_id;

        if(address->type == XBEE_16_BIT_BROADCAST)
        {
            buf[2] = 0xFF;
            buf[3] = 0xFF;
        }
        else
        {
            buf[2] = address->addr.network_address >> 8;
            buf[3] = address->addr.network_address & 0xFF;
        }

        buf[4] = option;

        int ret = xbee_start_frame(xbee, sizeof(buf)+data_size, &accum);
        if(ret != 0)
        {
            return ret;
        }

        ret = xbee_write_bytes(xbee, sizeof(buf), buf, &accum);
        if(ret != 0)
        {
            return ret;
        }
    }
    else if(address->type == XBEE_64_BIT || address->type == XBEE_64_BIT_BROADCAST)
    {
        uint8_t buf[11];
        buf[0] = XBEE_TRANSMIT;
        buf[1] = frame_id;

        if(address->type == XBEE_64_BIT)
        {
            for(size_t i = 0; i < 8; ++i)
            {
                buf[2+i] = (address->addr.address >> (64 - 8*(i+1))) & 0xFF;
            }
        }
        else
        {
            for(size_t i = 0; i < 6; ++i)
            {
                buf[2+i] = 0;
            }

            buf[8] = 0xFF;
            buf[9] = 0xFF;
        }

        buf[10] = option;

        int ret = xbee_start_frame(xbee, sizeof(buf)+data_size, &accum);
        if(ret != 0)
        {
            return ret;
        }

        ret = xbee_write_bytes(xbee, sizeof(buf), buf, &accum);
        if(ret != 0)
        {
            return ret;
        }
    }

    int ret = xbee_write_bytes(xbee, data_size, data, &accum);
    if(ret != 0)
    {
        return ret;
    }

    return xbee_finish_frame(xbee, accum);
}

int xbee_parse_frame(xbee_parsed_frame_t * parsed_frame,
        size_t frame_size, const void * frame)
{
    assert(parsed_frame);

    const uint8_t * b = frame;
    if(frame_size < 2)
    {
        return XBEE_WRONG_LENGTH_FOR_API;
    }

    memset(parsed_frame, 0, sizeof(parsed_frame));
    parsed_frame->api_id = b[0];

    uint64_t addr;
    switch(parsed_frame->api_id)
    {
    case XBEE_MODEM_STATUS:
    case XBEE_TRANSMIT_STATUS:
        if(frame_size != 2)
        {
            return XBEE_WRONG_LENGTH_FOR_API;
        }

        parsed_frame->frame.status = b[1];

        return 0;
        break;
    case XBEE_AT_RESPONSE:
        if(frame_size < 5)
        {
            return XBEE_WRONG_LENGTH_FOR_API;
        }

        parsed_frame->frame_id = b[1];

        parsed_frame->frame.at_command_response.at_command[0] = b[2];
        parsed_frame->frame.at_command_response.at_command[1] = b[3];
        parsed_frame->frame.at_command_response.at_command[2] = '\0';

        parsed_frame->frame.at_command_response.status = b[4];

        parsed_frame->frame.at_command_response.data_size = frame_size-5;
        parsed_frame->frame.at_command_response.data = &b[5];

        return 0;
        break;
    case XBEE_REMOTE_AT_RESPONSE:
        if(frame_size < 15)
        {
            return XBEE_WRONG_LENGTH_FOR_API;
        }

        parsed_frame->frame_id = b[1];

        addr = 0;
        for(size_t i = 0; i < 8; ++i)
        {
            addr |= b[2+i] << (64-8*(i-1));
        }
        parsed_frame->frame.at_command_response.responder_address = addr;

        parsed_frame->frame.at_command_response.responder_network_address |= b[10] << 8;
        parsed_frame->frame.at_command_response.responder_network_address |= b[11];


        parsed_frame->frame.at_command_response.at_command[0] = b[12];
        parsed_frame->frame.at_command_response.at_command[1] = b[13];
        parsed_frame->frame.at_command_response.at_command[2] = '\0';

        parsed_frame->frame.at_command_response.status = b[14];

        parsed_frame->frame.at_command_response.data_size = frame_size-15;
        parsed_frame->frame.at_command_response.data = &b[15];

        return 0;
        break;
    case XBEE_RECEIVE:
        if(frame_size < 11)
        {
            return XBEE_WRONG_LENGTH_FOR_API;
        }

        addr = 0;
        for(size_t i = 0; i < 8; ++i)
        {
            addr |= b[1+i] << (64-8*(i-1));
        }
        parsed_frame->frame.receive.responder_address = addr;

        parsed_frame->frame.receive.rssi = b[9];
        parsed_frame->frame.receive.options = b[10];

        parsed_frame->frame.receive.packet_size = frame_size-11;
        parsed_frame->frame.receive.packet_data = &b[11];

        return 0;
        break;
    case XBEE_RECEIVE_16_BIT:
        if(frame_size < 5)
        {
            return XBEE_WRONG_LENGTH_FOR_API;
        }

        parsed_frame->frame.receive.responder_network_address = b[1] << 8;
        parsed_frame->frame.receive.responder_network_address = b[2];

        parsed_frame->frame.receive.rssi = b[3];
        parsed_frame->frame.receive.options = b[4];

        parsed_frame->frame.receive.packet_size = frame_size-5;
        parsed_frame->frame.receive.packet_data = &b[5];

        return 0;
        break;
    default:
        return XBEE_UNKNOWN_API_ID;
    }
}
