#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "bitmap.h"


//コンストラクタの実装
Bitmap::Bitmap(){
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            bitmap[x][y] = BIT0;
        }     
    }
}
Bitmap::Bitmap(const char *ptr){
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            if(ptr[x*LEN +y] == '0'){
                bitmap[x][y] = BIT0;
            }
            else if(ptr[x*LEN +y] == '1'){
                bitmap[x][y] = BIT1;
            }
        }     
    }
}
void Bitmap::Init(){
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            bitmap[x][y] = BIT0;
        }     
    }
    return;
}


void Bitmap::assign(const char * ptr){
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            if(ptr[x*LEN +y] == '.'){
                bitmap[x][y] = BIT0;
            }
            else if(ptr[x*LEN +y] == '*'){
                bitmap[x][y] = BIT1;
            }
        }     
    }
}


void Bitmap::print(){
    std::cout << "---------bitmap---------";
    std::cout << "\n";
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            std::cout << bitmap[x][y];
            
        }
        //printf("%s",bitmap[x]);
        std::cout << "\n";
    }
    return;
}

void Bitmap::setRandom(){
    srand((unsigned int) time(NULL));
    int result;
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            result = rand() % 2;
            if(result == 0){
                bitmap[x][y] = BIT0;
            }
            else if(result == 1){
                bitmap[x][y] = BIT1;
            }
        }     
    }
    return;
}


void Bitmap::setRandomQ(){
    srand((unsigned int) time(NULL));
    int result;
    for(int x = 0;x<(int)LEN/2;x++){
        for(int y = 0;y<(int)LEN/2;y++){
            result = rand() % 2;
            if(result == 0){
                bitmap[x][y] = BIT0;
            }
            else if(result == 1){
                bitmap[x][y] = BIT1;
            }
        }     
    }
    return;
}

