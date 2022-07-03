#include "othello.h"
//コンストラクタの実装
Othello::Othello(){
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            m_board[x][y] = SPACE;
        }     
    }
    m_board[3][4] = m_board[4][3] = WHITE;
    m_board[4][4] = m_board[3][3] = BLACK;
}
Othello::Othello(const char *ptr){
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            if(ptr[x*LEN +y] == '1'){
                m_board[x][y] = 1;
            }
            else if(ptr[x*LEN +y] == '0'){
                m_board[x][y] = 0;
            }
        }     
    }
}
void Othello::Init(){
    for(int x = 0;x<LEN;x++){
        for(int y = 0;y<LEN;y++){
            m_board[x][y] = SPACE;
        }     
    }
    m_board[3][4] = m_board[4][3] = WHITE;
    m_board[4][4] = m_board[3][3] = BLACK;
}
        