#ifndef OPTICALROUTING_POINT_H
#define OPTICALROUTING_POINT_H
#include <iostream>
#include <cstdlib>

class Point {
public:
    Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    void print(std::ostream &out) const {
        out << "Center: (" << x << " " << y << ")" << std::endl;       
    }
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    bool operator < (const Point& other) const {
        if (x + y == other.x + other.y) {
            return x < other.x;
        }
        return (x + y) < (other.x + other.y);
    }
    bool operator > (const Point& other) const {
        if (x + y == other.x + other.y) {
            return x > other.x;
        }
        return (x + y) > (other.x + other.y);
    }
    double getX() const { return x; }
    double getY() const { return y; }
    void setX(double x) { this->x = x; }
    void setY(double y) { this->y = y; }

private:
    double x;
    double y;
};

#endif // OPTICALROUTING_POINT_H
