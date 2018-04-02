#ifndef SOFTWATCHDOG_H
#define SOFTWATCHDOG_H

class SoftWatchdog {
public:
    SoftWatchdog(unsigned int maxAgeMs);
    void feed();
    bool hungry();
private:
    unsigned long lastFed;
    unsigned int maxAge;
};

#endif
