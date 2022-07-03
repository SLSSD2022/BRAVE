
#include <iostream>
#include <stdlib.h>
#include "bitmap.h"


int main(){
    Bitmap MyBitmap;
    Bitmap YourBitmap("1111111110000011100001011000100110010001101000011100000111111111111111111000001110000101100010011001000110100001110000011111111111111111100000111000010110001001100100011010000111000001111111111111111110000011100001011000100110010001101000011100000111111111");
    YourBitmap.print();
    YourBitmap.Init();
    std::cout << "---------Initialized!---------";
    std::cout << "\n";
    YourBitmap.print();
    YourBitmap.setRandom();
    std::cout << "---------Randomized!---------";
    std::cout << "\n";
    YourBitmap.print();
    YourBitmap.setRandomQ();
    std::cout << "---------1/4Randomized!---------";
    std::cout << "\n";
    YourBitmap.print();
    YourBitmap.setRandomQ();
    std::cout << "---------1/4Randomized!---------";
    std::cout << "\n";
    YourBitmap.print();
    return 0;
}