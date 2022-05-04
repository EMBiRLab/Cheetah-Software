#ifndef IOCTL_WRAP
#define IOCTL_WRAP

#include <asm/termbits.h>
// #include <asm/termios.h>

int ioctl_wrap(int fd, unsigned long request, struct termios2* tty_ptr);

#endif