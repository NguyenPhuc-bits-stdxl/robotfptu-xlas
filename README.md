### Đây là file robot ESP32, điều khiển 2 servo, 2 motor 300RPM
## Quan trọng: Cài driver CP210x
Cài driver CP210x ở [đây](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads).

Tự xem cách cài driver CP210x cho Windows thông qua Device Manager (devmgmt-msc).

Còn với macOS, bấm vào file dmg.

## Cài đặt thư viện
Tải file .ZIP về và chép hết tất cả các file .h và .cpp vào:
* For Windows:
```
C:\Users\<your username>\Documents\Arduino\libraries
```
* For macOS:
```
/Users/<your username>/Documents/Arduino
```
Đây là thư mục của file ZIP tải về, các file được highlight là cần chép.

![image](https://github.com/user-attachments/assets/143ca932-63c3-424b-9da5-c88de498a35e)

**KHÔNG CẦN THIẾT NHƯNG KHUYẾN KHÍCH** làm theo, có thể để thư viện trong cùng folder và Arduino IDE có thể nhận thư mục được.

## Cài nhận diện ESP32 trong Boards Manager
Cài 2 bộ sau:
* **Arduino** ESP32 Boards by Arduino
* **esp32** by Espressif Systems

## Cách nạp
Mở Arduino IDE, chọn cổng, sẽ báo Unknown, bấm vào và lựa chọn bên Boards là **ESP32 Wrover Module**
Vào Tools và cài cấu hình sau:
* Flash Frequency: 40 MHz
* Flash Mode: QIO
* Upload Speed: 921600
* Programmer: Esptool

## Chú ý: Rút receiver của PS2 controller ra trước khi nạp (rút cục đen hoặc rút dây cáp SPI màu trắng cũng được)
## Tiếp...
Bấm **Upload** và chờ code nạp.

Có thể xem Serial Monitor sau khi nạp để check dữ liệu, để baudrate là **115200**.
