
#ifndef _Vector2_h_
#define _Vector2_h_

#include <iostream>

template<typename T>
class Vector2{
public:
    //constructors
    Vector2(T ix, T iy);
    Vector2(T ia);
    Vector2();
    
    //variables
    T x,y;
    
    //operators
    Vector2 operator+(const Vector2 &right) const;
    Vector2 operator-(const Vector2 &right) const;
    Vector2 operator*(const Vector2 &right) const;
    Vector2 operator*(const int &right) const;
    Vector2 operator/(const Vector2 &right) const;
    Vector2& operator+=(const Vector2 &right);
    Vector2& operator-=(const Vector2 &right);
    Vector2& operator*=(const Vector2 &right);
    Vector2& operator/=(const Vector2 &right);
    
    //fucntions
    void normalize();
    Vector2 normalized() const;
    T absolute() const;
    static T dotProduct(const Vector2 &left, const Vector2 &right);
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector2<T>& vector2);
#endif //_Vector2_hpp_