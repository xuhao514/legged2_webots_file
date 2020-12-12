#ifndef __LOG_H
#define __LOG_H
#include <iostream>
#include <fstream>

class LogFile
{
public:
    // LogFile(){
    //     if(!msg_file.is_open())
    //         msg_file.open("log_msg.txt", std::ios::trunc);  //log文件
    //     if(!error_file.is_open())
    //         error_file.open("log_error.txt", std::ios::trunc);  //错误信息文件
    // };
    // ~LogFile(){
    //     if (msg_file.is_open()) 
    //         msg_file.close();
    //     if (error_file.is_open()) 
    //         error_file.close();
    // };
   static void logMsg(const char* _head, double _dat){
       if(!msg_file.is_open())
            msg_file.open("log_msg.txt", std::ios::trunc);  //log文件
        msg_file<<_head<<" "<<_dat<<" ;"<<"\n";
    };
   static void logMsg(const char* _head, int _dat){
       if(!msg_file.is_open())
            msg_file.open("log_msg.txt", std::ios::trunc);  //log文件
        msg_file<<_head<<" "<<_dat<<" ;"<<"\n";
    };
  static void logEror(const char* _msg){
      if(!error_file.is_open())
            error_file.open("log_error.txt", std::ios::trunc);  //错误信息文件
        msg_file<<_msg<<"\n";
    };

private:
  static  std::ofstream msg_file;     //静态的成员变量要进行初始化
  static  std::ofstream error_file;

};

#endif