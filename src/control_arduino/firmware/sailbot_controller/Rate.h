#ifndef RATE_H
#define RATE_H

class Rate {
public:
    Rate(int);
    bool needsRun();
    void sleep();
    void setRate(int rate);
private:
    unsigned long periodUs;
    unsigned long lastUs;
};

#endif
