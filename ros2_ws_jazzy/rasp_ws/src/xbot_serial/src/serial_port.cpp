/**
 * @file serial_port.cpp
 * @author Davy Refss
 * @brief 串口通信
 * @note 
 */

/* include files */ 
#include <fcntl.h>      /* 文件控制 */
#include <unistd.h>     /* POSIX 操作系统 API 的核心头文件 */
#include <termios.h>    /* 终端（包括串口）输入输出的配置接口 */
#include <sys/select.h> /* 提供 select() 函数，用于 I/O 多路复用 */
#include <sys/ioctl.h>  /* ioctl(): TIOCEXCL 串口独占 */
#include <poll.h>       /* 提供 poll() 函数，用于 I/O 多路复用 */
#include <cstring>      /* std::memcpy */
#include <cerrno>       /* errno */ 
#include <algorithm>    /* std::min */
#include <stdexcept>    /* C++ 标准异常类 */
#include <iostream>
#include "xbot_serial/serial_port.hpp"

namespace xbot_serial {
/**
 * @brief 析构函数 关闭串口
 */    
SerialPort::~SerialPort() {
    close();
}

/**
 * @brief 将波特率转换为 termios 支持的速率
 * @param baud_rate 波特率
 * @param speed 速率
 * @return ture or false
 */
static bool BaudToSpeed(uint32_t baud_rate, speed_t &speed) {
    switch (baud_rate) {
        case 9600:      speed = B9600;      return true;       
        case 19200:     speed = B19200;     return true;
        case 38400:     speed = B38400;     return true;
        case 57600:     speed = B57600;     return true;
        case 115200:    speed = B115200;    return true;
        case 230400:    speed = B230400;    return true;
        case 460800:    speed = B460800;    return true;
        case 921600:    speed = B921600;    return true;
        case 1000000:   speed = B1000000;   return true;
        default:        return false;   
    }
}

/**
 * @brief 打开串口
 * @param port 串口名称
 * @param baud_rate 波特率
 * @return 成功返回 true, 失败返回 false 
 */
bool SerialPort::open(const std::string& port, uint32_t baud_rate) {
    if (fd_ >= 0) {
        close();
    }
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        /* cerr是一个与标准错误流相连的输出流，采用无缓冲模式*/
        std::cerr << "Failed to open serial port: " << port << ": " << std::strerror(errno) << std::endl;
        return false;
    }
    /* 独占打开: 已有进程占用时, 后续 open() 直接返回 EBUSY */
    if (ioctl(fd_, TIOCEXCL) != 0) {
        std::cerr << "Failed to set exclusive mode: " << std::strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    /* termios用于配置和控制终端的输入输出模式 */
    termios tty{};
    /* 获取串口属性 写到tty */
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "Failed to get serial port attributes: " << std::strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    speed_t speed = 0;
    if (!BaudToSpeed(baud_rate, speed)) {
        std::cerr << "Unsupported baud rate:" << baud_rate << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
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
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH); // flush input and output buffers
    return true;
}

/**
 * @brief 关闭串口
 */
void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

/**
 * @brief 判断串口是否打开
 * @return 打开返回 true, 否则返回 false
 */
bool SerialPort::IsOpen() const {
    return fd_ >= 0;
}

/**
 * @brief 设置读超时时间
 * 
 */
void SerialPort::SetTimeoutRead(int timeout) {
    timeout_read_ = std::max(timeout, 0);   
}

/**
 * @brief 设置写超时时间(发送缓冲满时等待可写的上限)
 * @param timeout 超时时间, 单位 ms
 */
void SerialPort::SetTimeoutWrite(int timeout) {
    timeout_write_ms_ = std::max(timeout, 0);
}

/**
 * @brief 写入数据
 * @param data 数据指针
 * @param size 数据长度
 * @return 成功返回写入的字节数, 失败返回 -1
 */
int SerialPort::write(const uint8_t* data, size_t size) {
    if (fd_ < 0) {
        std::cerr << "Serial port is not open." << std::endl;
        return -1;
    }
    size_t total = 0;
    while (total < size) {
        ssize_t n = ::write(fd_, data + total, size - total);
        if (n > 0) {
            total += static_cast<size_t>(n);   /* 短写: 继续写剩下的 */
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;                          /* 被信号打断: 重试 */
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            /* 发送缓冲满: 等可写, 最多 timeout_write_ms_ 毫秒 */
            struct pollfd pfd = { fd_, POLLOUT, 0 };
            int r = ::poll(&pfd, 1, timeout_write_ms_);
            if (r < 0 && errno == EINTR) {
                continue;
            }
            if (r <= 0) {                      /* 超时 */
                std::cerr << "Write timeout" << std::endl;
                break;
            }
            continue;
        }
        if (n == 0) {
            break;                             /* 防死循环 */
        }
        std::cerr << "Failed to write to serial port: " << std::strerror(errno) << std::endl;
        break;
    }
    return static_cast<int>(total);            /* 返回实际写出的字节数 */
}

/**
 * @brief 读取数据
 * @param data 数据指针
 * @param size 数据长度
 * @return 成功返回读取的字节数, 失败返回 -1
 */
int SerialPort::read(uint8_t* data, size_t size) {
    if (fd_ < 0) {
        std::cerr << "Serial port is not open." << std::endl;
        return -1;
    }

    /* select 被信号打断(EINTR)时要重试; 重试时必须重建 fd_set/timeval,
     * 因为 select 返回后会把它们改写成"剩余时间"和"就绪集合" */
    int ret = 0;
    do {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        timeval timeout{};
        timeout.tv_sec = timeout_read_ / 1000;
        timeout.tv_usec = (timeout_read_ % 1000) * 1000;

        ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    } while (ret < 0 && errno == EINTR);
    if (ret < 0) {
        std::cerr << "Select error: " << std::strerror(errno) << std::endl;
        return -1;
    } else if (ret == 0) {
        // Timeout
        return 0;
    }

    ssize_t bytes_read = 0;
    do {
        bytes_read = ::read(fd_, data, size);
    } while (bytes_read < 0 && errno == EINTR);
    if (bytes_read < 0) {
        /* select 报可读但 read 拿到 EAGAIN: 数据已被别的进程取走, 按"本次无数据"处理,
         * 否则会每秒刷屏报错 */
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        std::cerr << "Failed to read from serial port: " << std::strerror(errno) << std::endl;
        return -1;
    }
    return static_cast<int>(bytes_read);
}

}
