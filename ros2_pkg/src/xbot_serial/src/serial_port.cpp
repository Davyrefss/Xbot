#include <fcntl.h>      /* 文件控制 */
#include <unistd.h>     /* POSIX 操作系统 API 的核心头文件 */
#include <termios.h>    /* 终端（包括串口）输入输出的配置接口 */
#include <sys/select.h> /* 提供 select() 函数，用于 I/O 多路复用，可以同时监视多个文件描述符 */
#include <cstring>
#include <stdexcept>    /* C++ 标准异常类 */
#include <iostream>
#include "xbot_serial/serial_port.hpp"


namespace xbot_serial {
SerialPort::~SerialPort() {
    close();
}

static speed_t BaudToSpeed(uint32_t baud_rate) {
    switch (baud_rate) {
        case 9600:      return B9600;       /* B--termios 的波特率 */
        case 19200:     return B19200;
        case 38400:     return B38400;
        case 57600:     return B57600;
        case 115200:    return B115200;
        case 230400:    return B230400;
        case 460800:    return B460800;
        case 921600:    return B921600;
        case 1000000:   return B1000000;
        default:   
            /* throw --> 抛出一个异常 invalid_argument(无效参数类),传入参数 */     
            throw std::invalid_argument("Unsupported baud_rate: " + std::to_string(baud_rate));
    }
}

/* 将波特率转换为 termios 支持的速率 */
bool SerialPort::open(const std::string& port, uint32_t baud_rate) {
    /* ‘::’--全局作用域解析运算符(整个程序共用一个全局作用域，open函数重名了，C标准的头文件在include后放在了全局作用域，C++的头文件一般放在了std空间中) */
    /* port.c_str()-->将std::string转换为C风格字符串，O_RDWR | O_NOCTTY | O_NONBLOCK --> 读写|不要把该设备当作控制终端|非阻塞方式 */
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        /* cerr是一个与标准错误流相连的输出流，采用无缓冲模式*/
        std::cerr << "Failed to open serial port: " << port << ": " << std::strerror(errno) << std::endl;
        return false;
    }
    /* termios用于配置和控制终端的输入输出模式 */
    termios tty{};
    /* 获取串口属性 写到tty */
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "Failed to get serial port attributes: " << std::strerror(errno) << std::endl;
        ::close(fd_);
        return false;
    }

    speed_t speed = BaudToSpeed(baud_rate);
    /* 设置 termios 结构中输出波特率的函数 */
    cfsetospeed(&tty, speed);   
    cfsetispeed(&tty, speed);

    /* 8N1, 无流控，raw模式 */
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // no parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                        // no hardware flow control

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;                          // raw output
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); // raw input

    /* 非阻塞读， 用select控制超时 */
    tty.c_cc[VMIN] = 0;     // minimum number of characters to read
    tty.c_cc[VTIME] = 0;    // timeout in deciseconds (0 means no timeout)

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Failed to set serial port attributes: " << std::strerror(errno) << std::endl;
        ::close(fd_);
        return false;
    }

    tcflush(fd_, TCIOFLUSH); // flush input and output buffers
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

/* const 表示不修改调用它的那个 SerialPort 对象 */
bool SerialPort::IsOpen() const {
    return fd_ >= 0;
}

void SerialPort::SetTimeoutRead(int timeout) {
    timeout_read_ = timeout;
}

int SerialPort::write(const uint8_t* data, size_t size) {
    if (fd_ < 0) {
        std::cerr << "Serial port is not open." << std::endl;
        return -1;
    }
    ssize_t bytes_written = ::write(fd_, data, size);
    if (bytes_written < 0) {
        std::cerr << "Failed to write to serial port: " << std::strerror(errno) << std::endl;
        return -1;
    }
    /* static_cast 是 C++ 的显式类型转换运算符 */
    return static_cast<int>(bytes_written);
}

int SerialPort::read(uint8_t* data, size_t size) {
    if (fd_ < 0) {
        std::cerr << "Serial port is not open." << std::endl;
        return -1;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    timeval timeout{};
    timeout.tv_sec = timeout_read_ / 1000;
    timeout.tv_usec = (timeout_read_ % 1000) * 1000;

    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ret < 0) {
        std::cerr << "Select error: " << std::strerror(errno) << std::endl;
        return -1;
    } else if (ret == 0) {
        // Timeout
        return 0;
    }

    ssize_t bytes_read = ::read(fd_, data, size);
    if (bytes_read < 0) {
        std::cerr << "Failed to read from serial port: " << std::strerror(errno) << std::endl;
        return -1;
    }
    return static_cast<int>(bytes_read);
}

}
