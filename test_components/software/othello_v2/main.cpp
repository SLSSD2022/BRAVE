
#include <iostream>
#include <stdlib.h>
#include "othello.h"


int main(){
    Othello Myothello;
    Othello Yourothello("1111111110000011100001011000100110010001101000011100000111111111");
    Yourothello.print();
    Yourothello.Init();
    Yourothello.print();
    return 0;
}