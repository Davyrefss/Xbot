#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

/* include files */
#include <string>
#include <cstdint>
#include <vector>

namespace xbot_serial {
class SerialPort {
public:
    /** 构造函数在创建对象时被调用, '= default'表示请求编译器生成默认实现,只能用于特殊成员函数
     * 默认构造函数,拷贝构造函数,移动构造函数,拷贝赋值运算符,移动赋值运算符,析构函数,如果用户没有提供这些函数的定义,编译器会自动生成默认实现
     */ 
    SerialPort() = default; 
    /* 析构函数在对象销毁时被调用 */
    ~SerialPort();

    /* 禁止拷贝 */
    /** 拷贝构造函数, 指向的是一个不能修改的对象的引用，
     * ‘= delete’表示禁止拷贝, 例如->
     * SerialPort port1;
     * SerialPort port2 = port1; // 错误，禁止拷贝
     * SerialPort port3(port1);  // 错误，禁止拷贝
     */
    SerialPort(const SerialPort&) = delete;
    /**
     * 定义了一个重载赋值运算符函数
     * SerialPort port1;
     * SerialPort port2;
     * port2 = port1;(port2.operator=(port1)) // 错误，禁止赋值
     */
    SerialPort& operator=(const SerialPort&) = delete;
    /* open serial port */
    bool open(const std::string& port, uint32_t baud_rate);
    /* close serial port */
    void close();
    /* is it turned on */
    /**
     * const说明符，表示不能修改类对象的的成员变量
     */
    bool IsOpen() const;
    /* 写数据，返回写入的字节数 */
    int write(const uint8_t* data, size_t size);
    /* 读数据，返回读取的字节数 */
    int read(uint8_t* data, size_t size);
    /* 设置读超时 */
    void SetTimeoutRead(int timeout);

private:
    int fd_ = -1;   /* file descriptor */
    int timeout_read_ = 1000;   /* ms */
};

} /* namespace xbot_serial */


#endif /* SERIAL_PORT_HPP */
