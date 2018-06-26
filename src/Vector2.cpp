
#include "Vector2.h"
#include <cmath>

template<typename T>
Vector2<T>::Vector2(T ix, T iy): x(ix), y(iy){}
template<typename T>
Vector2<T>::Vector2(T ia): x(ia), y(ia){}
template<typename T>
Vector2<T>::Vector2(): x(0), y(0){}

template<typename T>
Vector2<T> Vector2<T>::operator+(const Vector2<T> &right) const{
    Vector2<T> temp(*this);
    temp += right;
    return temp;
}
template<typename T>
Vector2<T> Vector2<T>::operator-(const Vector2<T> &right) const{
    Vector2<T> temp(*this);
    temp -= right;
    return temp;
}
template<typename T>
Vector2<T> Vector2<T>::operator*(const Vector2<T> &right) const{
    Vector2<T> temp(*this);
    temp *= right;
    return temp;
}
template<typename T>
Vector2<T> Vector2<T>::operator*(const int &right) const{
    Vector2<T> temp(*this);
    temp.x *= right;
    temp.y *= right;
    return temp;
}

template<typename T>
Vector2<T> Vector2<T>::operator/(const Vector2<T> &right) const{
    Vector2<T> temp(*this);
    temp /= right;
    return temp;
}
template<typename T>
Vector2<T>& Vector2<T>::operator+=(const Vector2<T> &right){
    x += right.x;
    y += right.y;
    return *this;
}
template<typename T>
Vector2<T>& Vector2<T>::operator-=(const Vector2<T> &right){
    x -= right.x;
    y -= right.y;
    return *this;
}
template<typename T>
Vector2<T>& Vector2<T>::operator*=(const Vector2<T> &right){
    x *= right.x;
    y *= right.y;
    return *this;
}
template<typename T>
Vector2<T>& Vector2<T>::operator/=(const Vector2<T> &right){
    x /= right.x;
    y /= right.y;
    return *this;
}
template<typename T>
void Vector2<T>::normalize(){
    T lenght = sqrt(pow(x,2) + pow(y,2));
    x /= lenght;
    y /= lenght;
}
template<typename T>
Vector2<T> Vector2<T>::normalized() const{
    T lenght = sqrt(pow(x,2) + pow(y,2));
    return Vector2<T>(
                      x / lenght,
                      y / lenght);
}
template<typename T>
T Vector2<T>::absolute() const{
    return sqrt(pow(x,2) + pow(y,2));
}
template<typename T>
T Vector2<T>::dotProduct(const Vector2<T> &left, const Vector2<T> &right){
    return left.x * right.x + left.y * right.y;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector2<T>& vector2){
    os << '{' << vector2.x << ',' << vector2.y << '}';
    return os;
}

//compile the following classes and function using the templates

//Compile for double
template class Vector2<double>;
template std::ostream& operator<<(std::ostream& os, const Vector2<double>& vector2);
//Compile for float
template class Vector2<float>;
template std::ostream& operator<<(std::ostream& os, const Vector2<float>& vector2);