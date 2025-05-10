# BMP180 Linux Kernel Driver

## 1. Giới thiệu

Driver nhân Linux dành cho cảm biến nhiệt độ và áp suất BMP180 sử dụng giao tiếp I2C. Driver này cho phép người dùng đọc dữ liệu nhiệt độ và áp suất từ BMP180 thông qua giao tiếp ioctl từ không gian người dùng.

## 2. Tính năng

- Giao tiếp với BMP180 qua I2C.
- Đọc nhiệt độ thô (uncompensated temperature).
- Đọc áp suất thô (uncompensated pressure).
- Giao tiếp với không gian người dùng qua ioctl.
- Tạo node `/dev/bmp180`.

## 3. Cài đặt

### 3.1. Biên dịch driver

Tạo file `bmp180_driver.c` và viết mã driver.

Tạo Makefile:

```
obj-m += bmp180_driver.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
```

Chạy lệnh:

```
make
```

### 3.2. Nạp module

```
sudo insmod bmp180_driver.ko
```

### 3.3. Gỡ bỏ module

```
sudo rmmod bmp180_driver
```

## 4. Tạo file thiết bị

```
sudo mknod /dev/bmp180 c <major_number> 0
sudo chmod 666 /dev/bmp180
```

## 5. API ioctl hỗ trợ

```
#define BMP180_IOCTL_READ_TEMP _IOR('b', 1, int)
#define BMP180_IOCTL_READ_PRESSURE _IOR('b', 2, int)
```

## 6. Chương trình người dùng mẫu

```c
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

    if (ioctl(fd, BMP180_IOCTL_READ_TEMP, &temp) < 0) {
        perror("Failed to read temperature");
        close(fd);
        return -1;
    }
    printf("Temperature: %d\n", temp);

    if (ioctl(fd, BMP180_IOCTL_READ_PRESSURE, &pressure) < 0) {
        perror("Failed to read pressure");
        close(fd);
        return -1;
    }
    printf("Pressure: %d\n", pressure);

    close(fd);
    return 0;
}
```

## 7. Ghi chú sử dụng

- Nên đảm bảo thiết bị đã được khai báo đúng trong device tree hoặc qua I2C bus.
- Thời gian chờ sau lệnh đo là cần thiết để cảm biến có đủ thời gian xử lý.

## 8. Kiến trúc driver

- `probe`: Khởi tạo thiết bị và tạo device file.
- `remove`: Gỡ thiết bị và giải phóng tài nguyên.
- `ioctl`: Xử lý lệnh từ không gian người dùng.
- `read_raw_temp`, `read_raw_pressure`: Đọc dữ liệu từ cảm biến.

## 9. Kiểm thử

Sau khi nạp module và chạy chương trình người dùng mẫu, bạn nên thấy dữ liệu nhiệt độ và áp suất in ra màn hình.

## 10. VÍ DỤ ỨNG DỤNG

### 10.1. Trạm thời tiết đơn giản

#### 10.1.1. Sơ đồ kết nối
- BMP180 VCC → 3.3V Raspberry Pi
- BMP180 GND → GND
- BMP180 SDA → SDA (GPIO 2)
- BMP180 SCL → SCL (GPIO 3)

#### 10.1.2. Code ví dụ
- Sử dụng đoạn chương trình ở mục 6 để thu thập dữ liệu và hiển thị ra terminal hoặc ghi vào file log.

### 10.2. Đo độ cao cho dự án bay

#### 10.2.1. Sơ đồ kết nối
- Kết nối tương tự như trạm thời tiết.

#### 10.2.2. Code ví dụ
- Tính độ cao từ áp suất theo công thức:  
  ```c
  float altitude = 44330.0 * (1.0 - pow((pressure / 101325.0), 0.1903));
  ```

### 10.3. Các ứng dụng khác

- Giám sát môi trường.
- Đo áp suất không khí trong nhà thông minh.
- Gắn lên robot để cảm nhận môi trường xung quanh.

## 11. TÀI LIỆU THAM KHẢO

### 11.1. Datasheet của BMP180
- Bosch BMP180 Digital Pressure Sensor Datasheet  
  https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

### 11.2. Liên kết đến trang web/Repository của Driver
- https://github.com/yourusername/bmp180-linux-driver

### 11.3. Thông tin liên hệ
- Tác giả: Nguyễn Văn A  
- Email: nguyenvana@example.com  
- GitHub: https://github.com/yourusername