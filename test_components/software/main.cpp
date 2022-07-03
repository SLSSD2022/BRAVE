#include <iostream>
#include <stdlib.h>
#include "vector.h"
void createVector(){
    Vector somevec;
}

int main(){
    Vector myvector(5);
    Vector yourvector((int)9,(int)3);
    int d[] = {3,1,4,1};
    Vector hisvector(d,d+4);
    Vector hervector = hisvector;
    createVector();
    createVector();
    // myvector.m_size = 0;
    // myvector.m_capacity = 0;
    
    return 0;
}
