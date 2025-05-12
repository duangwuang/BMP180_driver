#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define BMP180_IOCTL_READ_TEMP_REAL  _IOR('b', 1, int)
#define BMP180_IOCTL_READ_PRESS_REAL _IOR('b', 2, int)

int main() {
    int fd;
    int temp, pressure;

    fd = open("/dev/bmp180", O_RDWR);
    if (fd < 0) {
        perror("Failed to open BMP180 device");
        return -1;
    }
    while (1){
    // Đọc nhiệt độ
    if (ioctl(fd, BMP180_IOCTL_READ_TEMP_REAL, &temp) < 0) {
        perror("Failed to read temperature");
        close(fd);
        return -1;
    }
    printf("Temperature: %d.%d °C\n", temp / 10, temp % 10);

    // Đọc áp suất
    if (ioctl(fd, BMP180_IOCTL_READ_PRESS_REAL, &pressure) < 0) {
        perror("Failed to read pressure");
        close(fd);
        return -1;
    }
    printf("Pressure: %d Pa\n", pressure);
    sleep(1);
    }
    
    close(fd);
    return 0;
}
