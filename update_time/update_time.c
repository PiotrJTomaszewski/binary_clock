#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

typedef unsigned char BYTE;
int main(int argc, char *argv[]) {
    char *device = "/dev/ttyUSB0";
    int opt;
    while ((opt = getopt(argc, argv, "hd:")) != -1) {
        switch(opt) {
            default:
            case 'h':
                puts("Clock time updater for my custom binary clock");
                puts("Options:");
                puts("-d\tSerial port to use. Can be ommited, defaults to /dev/ttyUSB0");
                puts("-h\tDisplay this message");
                return 0;
            case 'd':
                device = strdup(optarg);
                break;
        }
    }
    // Open serial port
    printf("Selected device: %s\n", device);
    int serial_port = open(device, O_RDWR);
    if (serial_port < 0) {
        printf("Error %d while opening serial port: %s\n", errno, strerror(errno));
        close(serial_port);
        return -1;
    }
    puts("Connection with the device opened");
    // Configure serial connection
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %d from tcgetattr: %s\n", errno, strerror(errno));
        close(serial_port);
        return -1;
    }
    tty.c_cflag &= ~PARENB;  // Disable parity bit
    tty.c_cflag &= ~CSTOPB;  // Use 1 stop bit
    tty.c_cflag |= CS8;      // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable flow control
    tty.c_cflag |= CLOCAL;   // Ignore modem status line
    tty.c_cflag |= CREAD;    // Enable reading
    tty.c_lflag &= ~ICANON;  // Disable canonical mode
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ISIG;    // Disable interperetation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Get raw data from serial device
    tty.c_oflag &= ~(OPOST | ONLCR); // Send raw data to serial device
    tty.c_cc[VTIME] = 100;   // Set timeout to 10s
    tty.c_cc[VMIN]  = 1;     // Wait for 1 byte of data
    // Set baudrate
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    // Save new settings
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %d from tcsetattr: %s\n", errno, strerror(errno));
        close(serial_port);
        return -1;
    }
    // Get current time
    struct tm *current_time;
    time_t raw_time;
    time(&raw_time);
    current_time = localtime(&raw_time);
    // Write time to serial device
    BYTE message[5];
    // Set control bytes
    message[0] = 0xca;
    message[1] = 0xfe;
    message[2] = (BYTE)current_time->tm_hour;
    message[3] = (BYTE)current_time->tm_min;
    message[4] = (BYTE)current_time->tm_sec;
    printf("Setting time: %d:%d:%d\n", message[2], message[3], message[4]);
    if (write(serial_port, message, 5) < 0 ) {
        printf("Error %d while writing to serial device: %s\n", errno, strerror(errno));
        close(serial_port);
        return -1;
    }
    puts("New time sent to the device, waiting for acknowledgment.");
    // Get acknowledgement from the device
    BYTE read_byte;
    int n = read(serial_port, &read_byte, 1);
    if (n<0) {
        printf("Error %d while reading from the serial device: %s", errno, strerror(errno));
        close(serial_port);
        return -1;
    }
    if (read_byte != 0x00) {
        printf("Error: wrong message was sent to the serial device. Device returned code: %x\n", read_byte);
        close(serial_port);
        return -1;
    }
    printf("New time successfully set\n");
    close(serial_port);
    puts("Connection with the device closed");
    return 0;
}
