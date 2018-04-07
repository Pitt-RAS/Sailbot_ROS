#ifndef WATCHDOG_H
#define WATCHDOG_H

class Watchdog{
public:
  Watchdog(unsigned int maxAgeMs);
  void feed();
  bool hungry();
  void refresh();
private:
  unsigned long lastFed;
  unsigned int maxAge;
};

#endif