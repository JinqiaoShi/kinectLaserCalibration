#ifndef SAFE_H
#define SAFE_H

#include <string>

class Logger{
public:
   static Logger* Instance();
   bool openLogFile(std::string logFile);
   void writeToLogFile();
   bool closeLogFile();

private:
   Logger(){};  // Private so that it can  not be called
   Logger(Logger const&){};             // copy constructor is private
   Logger& operator=(Logger const&){};  // assignment operator is private
   static Logger* m_pInstance;
};

#endif
