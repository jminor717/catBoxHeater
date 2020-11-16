// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// 20, 24, 28, 29, 30, 31 are not exposed on the qfn package
// 0, 2, 5, 12, 15 can affect boot sequence if used improperly
// 6, 7, 8, 9, 10, 11 are connected to internal spi flash
// 32, 33 are connected to intrenal crystal and cannot be used
// 34, 35, 36, 39 are input only
// 1, 3, 5, 14, 15 output pwm durring boot/reboot, do not tie between boards

//unused input:
//FW1:18013 FW2:18201 FW3:18646 RW1:16514 RW2:14434 RW3:16147

#define PrimarySPI_MISO 22
#define PrimarySPI_MOSI 23
#define PrimarySPI_SCLK 21
#define PrimarySPI_SS 19
#define DReady 18
#define Reset 17


#define zcdPin 34

#define FrontHeaterCTRL 26
#define RearHeaterCTRL 27

#define AmbientTemp 4
#define FrontTemp 13
//config

#define WIFIapssid "catFi"
#define WIFIssid "Pretty fly for a wifi"
#define WIFIappassword "123456789"
#define WIFIpassword "JRMinor1!"

#define WIFI_HOST "catbox"
