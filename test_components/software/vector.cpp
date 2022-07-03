#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "vector.h"

Vector::Vector()
    :m_data(new int[INI_SIZE])
    ,m_size(0)
    ,m_capacity(INI_SIZE){
}

Vector::Vector(int len,int d)
    :m_data(new int[len])
    ,m_size(len)
    ,m_capacity(len*2)
{
    for(int i = 0;i < len;i++){
        m_data[i] = d;
    };
}

Vector::Vector(int *first,int* last)
    :m_data(new int[last - first])
    ,m_size(last - first)
    ,m_capacity((last - first)*2)
{
    for(int i = 0;i < m_size;i++){
        m_data[i] = *(first+i);
    };
}

Vector::Vector(const Vector &x)
    :m_data(new int[x.m_size])
    ,m_size(x.m_size)
    ,m_capacity(x.m_capacity)
{
    for(int i = 0;i < m_size;i++){
        m_data[i] = x.m_data[i];
    };
}

Vector::~Vector(){
    delete [] this->m_data;
}

