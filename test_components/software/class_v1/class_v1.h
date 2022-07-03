#include <string>
using namespace std;
class Person{
public:
    Person();
    Person(string a, double b , double c);
    Person(const Person &copy);
private:
    string name;
    double height;
    double weight;
};