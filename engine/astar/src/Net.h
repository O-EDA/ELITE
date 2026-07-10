#ifndef OPTICALROUTING_NET_H
#define OPTICALROUTING_NET_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include "Point.h"

class Net{
    public:
    int RipNum = 0;
    double dx = 0;
    double dy = 0;
    std::vector<Point> path;
    
    Net(const std::string& name, const Point source, const Point target): 
        name(name), source(source), target(target) {}

    Net(const Point& source, const Point& target): 
        Net("", source, target) {} // Delegates to the main constructor with an empty name

    double HPWL() const {
        return std::abs(source.getX() - target.getX()) + std::abs(source.getY() - target.getY());
    }
    void print(std::ostream &out) const {
        out << "Net: " << name << " HPWL: " << HPWL() << std::endl;
        source.print(out);
        target.print(out);
    }

    std::string getName() const { return name; }
    Point getSource() const { return source; }
    Point getTarget() const { return target; }
    bool getIsRouted() const { return isRouted; }
    int getRipNum() const { return RipNum; }
    std::vector<Point> getPath() const { return path; }
    int getSourceDirection() const { return sourceDirection; }
    int getTargetDirection() const { return targetDirection; }

    void setIsRouted(bool isRouted) { this->isRouted = isRouted; }
    void Ripup() { RipNum++; }
    void setSource(Point source) { this->source = source; }
    void setTarget(Point target) { this->target = target; }
    void setPath(std::vector<Point> path) { this->path = path; }
    void setSourceDirection(int direction) { this->sourceDirection = direction; }
    void setTargetDirection(int direction) { this->targetDirection = direction; }

    private:
    std::string name;
    Point source;
    Point target;
    bool isRouted = false;
    int sourceDirection = -1;
    int targetDirection = -1;
};

#endif // OPTICALROUTING_NET_H
