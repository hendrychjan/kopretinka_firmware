#pragma once

#include <Arduino.h>

template <uint16_t MAX_SIZE>
class Text {
public:
  Text() {
    clear();
  }

  Text(const char* str) {
    copy(str);
  }

  void clear() {
    buffer[0] = '\0';
  }

  void copy(const char* str) {
    strncpy(buffer, str, MAX_SIZE);
    buffer[MAX_SIZE] = '\0';
  }

  void copy(const Text& other) {
    strncpy(buffer, other.buffer, MAX_SIZE);
    buffer[MAX_SIZE] = '\0';
  }

  Text& operator=(const char* str) {
    copy(str);
    return *this;
  }

  Text& operator=(const Text& other) {
    copy(other);
    return *this;
  }

  const char* c_str() const {
    return buffer;
  }

  uint16_t length() const {
    return strlen(buffer);
  }

  uint16_t size() const {
    return sizeof(buffer);
  }

private:
  char buffer[MAX_SIZE + 1];
};
