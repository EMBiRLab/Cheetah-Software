#include <sys/ioctl.h>

int ioctl_wrap(int fd, unsigned long request, struct termios2* tty_ptr) {
    return ioctl(fd, request, tty_ptr);
}