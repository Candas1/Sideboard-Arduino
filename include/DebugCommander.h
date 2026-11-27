// Minimal command parser for the debug console (replaces SimpleFOC Commander).
// A line "<letter>[args]\n" calls the callback registered for <letter>;
// "?" lists the commands.
#ifndef DEBUG_COMMANDER_H
#define DEBUG_COMMANDER_H

#include <Arduino.h>

typedef void (*CommandCallback)(char *args);

class DebugCommander {
  public:
    DebugCommander(Stream &stream) : _stream(stream) {}

    void add(char id, CommandCallback cb, const char *label) {
      if (_count >= MAX_COMMANDS) return;
      _ids[_count] = id;
      _cbs[_count] = cb;
      _labels[_count] = label;
      _count++;
    }

    void run() {
      while (_stream.available()) {
        char c = (char)_stream.read();
        if (c == '\r') continue;
        if (c != '\n') {
          if (_len < sizeof(_buf) - 1) _buf[_len++] = c;
          continue;
        }
        _buf[_len] = '\0';
        if (_len > 0) dispatch();
        _len = 0;
      }
    }

  private:
    static const uint8_t MAX_COMMANDS = 10;

    void dispatch() {
      if (_buf[0] == '?') {
        for (uint8_t i = 0; i < _count; i++) {
          _stream.print(_ids[i]);
          _stream.print(F(": "));
          _stream.println(_labels[i]);
        }
        return;
      }
      for (uint8_t i = 0; i < _count; i++) {
        if (_ids[i] == _buf[0]) { _cbs[i](&_buf[1]); return; }
      }
      _stream.println(F("Unknown command, type ? for help"));
    }

    Stream &_stream;
    char _buf[32];
    uint8_t _len = 0;
    uint8_t _count = 0;
    char _ids[MAX_COMMANDS];
    CommandCallback _cbs[MAX_COMMANDS];
    const char *_labels[MAX_COMMANDS];
};

#endif // DEBUG_COMMANDER_H
