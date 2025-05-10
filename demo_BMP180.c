#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define BMP180_IOCTL_READ_TEMP _IOR('b', 1, int)
#define BMP180_IOCTL_READ_PRESSURE _IOR('b', 2, int)

int main() {
    int fd;
    int temp, pressure;

    fd = open("/dev/bmp180", O_RDWR);
    if (fd < 0) {
        perror("Failed to open BMP180 device");
        return -1;
    }

    // Đọc nhiệt độ
    if (ioctl(fd, BMP180_IOCTL_READ_TEMP, &temp) < 0) {
        perror("Failed to read temperature");
        close(fd);
        return -1;
    }
    printf("Temperature: %d\n", temp);

    // Đọc áp suất
    if (ioctl(fd, BMP180_IOCTL_READ_PRESSURE, &pressure) < 0) {
        perror("Failed to read pressure");
        close(fd);
        return -1;
    }
    printf("Pressure: %d\n", pressure);

    close(fd);
    return 0;
}
