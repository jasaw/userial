/*
 *    Filename: userial.c
 * Description: micro serial
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 jasaw
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/stat.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>


static const char *progname = "";
static unsigned char terminate = 0;
static const char *device_path = NULL;
static unsigned int baudrate = 38400;
static unsigned char flowcontrol = 0;
static unsigned char csize = 8;
static unsigned char twostopbits = 0;


#define max(a,b) ( (a > b) ? a : b )


static void serial_flush_input(int ttyfd)
{
    tcflush(ttyfd, TCIFLUSH);
}


//static void serial_sync_output(int ttyfd)
//{
//    tcdrain(ttyfd);
//}


static int set_non_block(int fd, unsigned char enable)
{
    int result = 0;
    int flags = fcntl(fd, F_GETFL);
    if (flags >= 0)
    {
        if (enable)
            flags |= O_NONBLOCK;
        else
            flags &= ~O_NONBLOCK;
        if (fcntl(fd, F_SETFL, flags) < 0)
            result = -1;
    }
    else
        result = -1;
    return result;
}


static int configure_serial_port(int fd)
{
    struct termios newtio;

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = CLOCAL | CREAD;

    switch (baudrate)
    {
        case 50:     newtio.c_cflag |= B50;     break;
        case 75:     newtio.c_cflag |= B75;     break;
        case 110:    newtio.c_cflag |= B110;    break;
        case 134:    newtio.c_cflag |= B134;    break;
        case 150:    newtio.c_cflag |= B150;    break;
        case 200:    newtio.c_cflag |= B200;    break;
        case 300:    newtio.c_cflag |= B300;    break;
        case 600:    newtio.c_cflag |= B600;    break;
        case 1200:   newtio.c_cflag |= B1200;   break;
        case 1800:   newtio.c_cflag |= B1800;   break;
        case 2400:   newtio.c_cflag |= B2400;   break;
        case 4800:   newtio.c_cflag |= B4800;   break;
        case 9600:   newtio.c_cflag |= B9600;   break;
        case 19200:  newtio.c_cflag |= B19200;  break;
        case 38400:  newtio.c_cflag |= B38400;  break;
        case 57600:  newtio.c_cflag |= B57600;  break;
        case 115200: newtio.c_cflag |= B115200; break;
        case 230400: newtio.c_cflag |= B230400; break;
        default: fprintf(stderr, "Error: invalid baudrate %d\n", baudrate); return -1;
    }
    switch (csize)
    {
        case 5: newtio.c_cflag |= CS5; break;
        case 6: newtio.c_cflag |= CS6; break;
        case 7: newtio.c_cflag |= CS7; break;
        case 8: newtio.c_cflag |= CS8; break;
        default: fprintf(stderr, "Error: invalid character size %d\n", csize); return -1;
    }
    if (flowcontrol)
        newtio.c_cflag |= CRTSCTS;
    if (twostopbits)
        newtio.c_cflag |= CSTOPB;

    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;

    // inter-character timer unused
    newtio.c_cc[VTIME] = 0;
    // no blocking read
    newtio.c_cc[VMIN] = 0;

    serial_flush_input(fd);
    tcsetattr(fd, TCSANOW, &newtio);

    return 0;
}



static void handle_terminate_signal(int sig)
{
    if ((sig == SIGTERM) || (sig == SIGINT))
        terminate = 1;
}


static void copy_fd_data(int dst_fd, int src_fd)
{
    ssize_t ret;
    char buffer[64];
    while (!terminate)
    {
        ret = read(src_fd, buffer, sizeof(buffer));
        if (ret > 0)
            write(dst_fd, buffer, ret);
        else
            break;
    }
}


static void forward(int fd)
{
    fd_set read_set;
    int maxfd;
    int status;

    while (!terminate)
    {
        maxfd = -1;
        FD_ZERO(&read_set);

        FD_SET(STDIN_FILENO, &read_set);
        maxfd = max(maxfd, STDIN_FILENO);
        FD_SET(fd, &read_set);
        maxfd = max(maxfd, fd);

        status = select(maxfd + 1, &read_set, NULL, NULL, NULL);
        if (status < 0)
        {
            int error_code = errno;
            // select was interrupted by a signal
            if (error_code != EINTR)
                fprintf(stderr, "Error: select() returned error: %s\n", strerror(error_code));
        }
        else
        {
            if (FD_ISSET(fd, &read_set))
                copy_fd_data(STDOUT_FILENO, fd);
            if (FD_ISSET(STDIN_FILENO, &read_set))
                copy_fd_data(fd, STDIN_FILENO);
        }
    }
}


void syntax()
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "%s [options]\n", progname);
    fprintf(stderr, "\n");
    fprintf(stderr, "options:\n");
    fprintf(stderr, " -D <path>     device path\n");
    fprintf(stderr, " -b <rate>     baudrate (default %d)\n", baudrate);
    fprintf(stderr, " -f            enable hardware flow control\n");
    fprintf(stderr, " -c <5|6|7|8>  character size (default %d)\n", csize);
    fprintf(stderr, " -s            set 2 stop bits (default 1 stop bit)\n");
    fprintf(stderr, " -h            display this information\n");
    fprintf(stderr, "\n");
    exit(EXIT_FAILURE);
}


int main(int argc, char *argv[])
{
    int ret = 0;
    struct stat st;
    int fd = -1;
    int status = 0;
    int opt;

    progname = argv[0];

    while ((opt = getopt (argc, argv, "D:b:fc:sh")) != -1)
    {
        switch (opt)
        {
            case 'D': device_path = optarg; if (strlen(device_path) == 0) syntax(); break;
            case 'b': baudrate = atoi(optarg); if (baudrate == 0) syntax(); break;
            case 'f': flowcontrol = 1; break;
            case 'c': csize = atoi(optarg); if ((csize < 5) || (csize > 8)) syntax(); break;
            case 's': twostopbits = 1; break;
            case 'h': // fall through
            default:
                syntax();
                break;
        }
    }

    if (device_path == NULL)
        syntax();

    if (stat(device_path, &st) != 0)
    {
        fprintf(stderr, "Error: %s does not exist\n", device_path);
        ret = EXIT_FAILURE;
        goto main_init_error;
    }

    fd = open(device_path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
    if (fd < 0)
    {
        fprintf(stderr, "Error: unable to open %s\n", device_path);
        ret = EXIT_FAILURE;
        goto main_init_error;
    }

    if (configure_serial_port(fd) != 0)
    {
        fprintf(stderr, "Error: unable to configure %s\n", device_path);
        ret = EXIT_FAILURE;
        goto main_init_error;
    }

    // set standard file descriptors to non-blocking
    set_non_block(STDIN_FILENO, 1);
    set_non_block(STDOUT_FILENO, 1);
    set_non_block(STDERR_FILENO, 1);

    signal(SIGINT, handle_terminate_signal);
    signal(SIGTERM, handle_terminate_signal);

    forward(fd);

main_init_error:
    if (fd >= 0)
        close(fd);
    return ret;
}
