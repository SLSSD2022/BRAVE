// #define othelloLen 8
class Othello{
    static const int LEN = 8;
    static const int SPACE = 0;
    static const int WHITE = 1;
    static const int BLACK = 2;
public:
    Othello();
    Othello(const char *ptr);
    void Init();
private:
    char m_board[LEN][LEN];
};