#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

#include "xbee.h"

int write_fd(void * ptr, const void *buf, size_t nbyte)
{
    int ret = write(*((int*)ptr), buf, nbyte);
    if(ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
        return 0;
    }

    return ret;
}

int read_fd(void * ptr, void *buf, size_t nbyte)
{
    int ret = read(*((int*)ptr), buf, nbyte);
    if(ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
        return 0;
    }

    return ret;
}

void hexdump(size_t n, void * ptr)
{
    uint8_t * b = ptr;

    for(size_t i = 0; i < n; ++i)
    {
        printf("%02x", b[i]);
        if((i+1)%8 == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}


void test_xbee(xbee_interface_t * xbee)
{
    char buf[1];
    int ret = xbee_at_command(xbee, 1, "BD", 0, buf);
    if(ret < 0)
    {
        printf("Error writing frame, ret = %d, errno = %d\n", ret, errno);
        return;
    }

    sleep(1);

    uint8_t frame[XBEE_MAX_FRAME_SIZE];
    ret = xbee_recv_frame(xbee, sizeof(frame), frame);
    if(ret < 0)
    {
        printf("Error getting frame, ret = %d, errno = %d\n", ret, errno);
        return;
    }

    printf("Got frame of %d length\n", ret);
    hexdump(ret, frame);

    char set_baud[1] = {4};
    ret = xbee_at_command(xbee, 1, "BD", sizeof(set_baud), set_baud);
    if(ret < 0)
    {
        printf("error writing frame, ret = %d, errno = %d\n", ret, errno);
        return;
    }

    sleep(1);

    ret = xbee_recv_frame(xbee, sizeof(frame), frame);
    if(ret < 0)
    {
        printf("error getting frame, ret = %d, errno = %d\n", ret, errno);
        return;
    }

    printf("Got frame of %d length\n", ret);
    hexdump(ret, frame);

    ret = xbee_at_command(xbee, 1, "BD", 0, buf);
    if(ret < 0)
    {
        printf("error writing frame, ret = %d, errno = %d\n", ret, errno);
        return;
    }

    sleep(1);

    ret = xbee_recv_frame(xbee, sizeof(frame), frame);
    if(ret < 0)
    {
        printf("error getting frame, ret = %d, errno = %d\n", ret, errno);
        return;
    }

    printf("Got frame of %d length\n", ret);
    hexdump(ret, frame);
}

int main(int argc, char * argv[])
{
    int fd;
    xbee_uart_interface_t uart = {
        .ptr = &fd,
        .write = write_fd,
        .read = read_fd,
        .sleep = sleep,
    };

    uint8_t buf[XBEE_REC_BUF_SIZE];

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0)
    {
        printf("fd = %d, errno = %d, strerror = %s\n", fd, errno, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf("error %d from tcgetattr\nstrerror = %s\n", errno, strerror(errno));
        return -1;
    }

    int speed = B19200;
    if(cfsetospeed (&tty, speed) != 0)
    {
        printf("error %d from cfsetospeed\nstrerror = %s\n", errno, strerror(errno));
        return -1;
    }
    if(cfsetispeed (&tty, speed) != 0)
    {
        printf("error %d from cfsetispeed\nstrerror = %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf("error %d from tcsetattr\nstrerror = %s\n", errno, strerror(errno));
        return -1;
    }

    xbee_interface_t xbee;
    int ret = xbee_open(&xbee, &uart, sizeof(buf), buf);
    if(ret != 0)
    {
        printf("ret = %d, errno = %d\n", ret, errno);
        return -1;
    }

    test_xbee(&xbee);

    ret = close(fd);
    if(ret != 0)
    {
        printf("ret = %d, errno = %d\n", ret, errno);
    }

    return 0;
}
