// #define othelloLen 8
class Othello{
    static const int LEN = 8;
    static const char SPACE = '0';
    static const char WHITE = '1';
    static const char BLACK = '2';
public:
    Othello();
    Othello(const char *ptr);
    void Init();
    void print();
private:
    char m_board[LEN][LEN];
};