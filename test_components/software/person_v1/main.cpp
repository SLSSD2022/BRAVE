
#include <stdio.h>
#include <stdlib.h>
#include <string>
using namespace std;
#include "person.h"


int main(){
    Person Myperson("Kazuki",150,90);
    Person Yourperson(Myperson);
    string str ="toma";
    double BMI = Myperson.BMI();
    return 0;
}