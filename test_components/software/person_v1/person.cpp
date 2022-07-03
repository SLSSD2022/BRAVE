#ifndef PERSON_H
#define PERSON_H

#include "person.h"
#include <string>
using namespace std;
//コンストラクタの実装
Person::Person():name("Toma"),height(176),weight(65){
}
Person::Person(string a, double b, double c): name(a),height(b),weight(c){
}
Person::Person(const Person &copy):name(copy.name),height(copy.height),weight(copy.weight){
}
double Person::BMI() const{
    return weight/(height * height)*10000;
}

#endif