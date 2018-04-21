#include <iostream>
#include <unistd.h>
#include "XbeeReceiver.h"

int main() {
    XbeeReceiver xbee;
    printf("Starting...\n");
    while ( 1 ) {
        xbee.update();
    }
    return 0;
}
