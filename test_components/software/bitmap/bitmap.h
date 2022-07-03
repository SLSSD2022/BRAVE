// #define BitmapLen 16
class Bitmap{
    static const int LEN = 16;
    static const char BIT0 = '.';
    static const char BIT1 = '*';
public:
    Bitmap();
    Bitmap(const char *ptr);
    void Init();
    void print();
    void setRandom();
    void setRandomQ();
    void assign(const char *ptr);
private:
    char bitmap[LEN][LEN];
};