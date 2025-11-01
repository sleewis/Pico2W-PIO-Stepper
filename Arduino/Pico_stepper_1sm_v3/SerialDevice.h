// SerialDevice.h
#pragma once
#include <Arduino.h>
#include <utility>   // for std::forward

class SerialDevice {
public:
  explicit SerialDevice(Stream& s) : io(&s) {}
  void setOutput(Stream& s) { io = &s; }

  // Forwarders: match all Arduino Print/Stream overloads,
  // including print(double,int) and println(double,int)
  template<typename... Args>
  size_t print(Args&&... args) {
    return io->print(std::forward<Args>(args)...);
  }

  template<typename... Args>
  size_t println(Args&&... args) {
    return io->println(std::forward<Args>(args)...);
  }

  // Read helpers (optional)
  int available() { return io->available(); }
  int read()      { return io->read(); }
  int peek()      { return io->peek(); }
  void flush()    { io->flush(); }

private:
  Stream* io;
};

// In one .cpp only:
extern SerialDevice SD;
