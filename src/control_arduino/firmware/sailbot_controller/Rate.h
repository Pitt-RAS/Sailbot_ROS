#ifndef RATE_H
#define RATE_H

class Rate {
public:
    Rate(int);
    bool needsRun();
    void sleep();
    
private:
    unsigned long periodUs;
    unsigned long lastUs;
};

#endif
