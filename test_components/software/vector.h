#ifndef VECTOR_H
#define VECTOR_H
// #define BitmapLen 16
class Vector{
public:
    enum{
        INI_SIZE = 8, //クラス固有の定数は enum で宣言するのが普通です。
    };
public:
    Vector();
    Vector(int len,int d = 0);
    Vector(int* first,int* last);
    Vector(const Vector &x);
    ~Vector();
    bool isEmptuy();
    int size();
private:
    int* m_data; //アロケートされたデータ領域へのポインタ
    int m_size; // データ領域に入っている要素数
    int m_capacity; // アロケートされたデータ領域サイズ
};

#endif