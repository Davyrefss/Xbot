/**
 * @file serial_port.hpp
 * @author Davy Refss
 * @brief 串口通信类
 * @note
 */

#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

/* include files */
#include <string>
#include <cstdint>
#include <vector>

namespace xbot_serial {
class SerialPort {
public:
    SerialPort() = default; 
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort &operator=(const SerialPort&) = delete;
    
    bool open(const std::string& port, uint32_t baud_rate); /* open serial port */
    void close();   /* close serial port */
    bool IsOpen() const;    /* is it turned on */
    int write(const uint8_t* data, size_t size);    /* 写数据，返回写入的字节数 */
    int read(uint8_t* data, size_t size);   /* 读数据，返回读取的字节数 */
    void SetTimeoutRead(int timeout);   /* 设置读超时 */
    void SetTimeoutWrite(int timeout);  /* 设置写超时 */

private:
    int fd_ = -1;   /* file descriptor */
    int timeout_read_ = 1000;   /* ms */
    int timeout_write_ms_ = 100;    /* ms, 写阻塞上限 */
};

} /* namespace xbot_serial */


#endif /* SERIAL_PORT_HPP */
