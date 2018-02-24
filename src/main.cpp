#include <mbed.h>

DigitalOut myled(P0_21);

int main() {

    // put your setup code here, to run once:

    while(1) {
        // put your main code here, to run repeatedly:
        myled = 1;
        wait(1);
        myled = 0;
        wait(1);
    }
}
