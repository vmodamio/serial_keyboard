/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <stdio.h>
#include <stdint.h>

uint16_t rowstat;
uint16_t mask;
uint64_t keydown;
#define NROWS 9
#define RMASK (2<<NROWS)-1
uint8_t colk = 1;

int main()
{
    keydown = 0b000010000 << (colk*NROWS); // second row in second column
    rowstat = 0b000000000;
    mask = rowstat ^ ((keydown & ((RMASK)<<(NROWS*colk)))>>(NROWS*colk)); 
    
    if (mask) {
        printf("Keyboard changed: \r\n");
        printf("before: %d \r\n", (int)((keydown & ((RMASK)<<(NROWS*colk)))>>(NROWS*colk)));
        printf("after: %d \r\n", rowstat);
        if (mask & (mask-1)) { // mask is not power of 2 (single bit)
            printf("More than one key changed!\r\n");
        }
        else {
            printf("One key changed: %d\r\n", mask);
            keydown ^= mask << (NROWS*colk);
        }
    }
    else {
        printf("No changes\r\n");
    }

    mask = rowstat ^ ((keydown & ((RMASK)<<(NROWS*colk)))>>(NROWS*colk)); 
    if (mask) {
        printf("Keyboard changed: \r\n");
        printf("before: %d \r\n", (int)((keydown & ((RMASK)<<(NROWS*colk)))>>(NROWS*colk)));
        printf("after: %d \r\n", rowstat);
        if (mask & (mask-1)) { // mask is not power of 2 (single bit)
            printf("More than one key changed!\r\n");
        }
        else {
            printf("One key changed: %d \r\n", mask);
            keydown ^= mask << (NROWS*colk);
        }
    }
    else {
        printf("No changes\r\n");
    }


    return 0;
}

