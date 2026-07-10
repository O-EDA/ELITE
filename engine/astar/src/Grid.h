#ifndef OPTICALROUTING_GRID_H
#define OPTICALROUTING_GRID_H

#include "Point.h"
#include <vector>
#include <map>
#include <string>

class Grid {
public:
    struct GlobalBin{
        Point center;
        double length = 0;
        double historyCost = 0;
        std::map<std::string, int> pattern;

        GlobalBin() : center(0, 0) { 
        init();
        }
        GlobalBin(double x, double y) : center(x, y) {
            init(); 
        }
        void init()   {
            for (int i = 0; i <= 8; ++i) {
                for (int j = i + 1; j <= 8; ++j) {
                    std::string key = std::to_string(i) + std::to_string(j);
                    pattern[key] = 0;
                }
            }

        }
    };

    Grid(double totalWidth, double totalHeight);

    void getCentralPoints();
    void setToCellCenter(Point& point) const;
    void setToCellCenterIdx(Point& point) const;

    int getRows() const { return rows; }
    int getCols() const { return cols; }
    std::vector<std::vector<GlobalBin>>& getGgrid() { return Ggrid; }

private:
    double totalWidth;
    double totalHeight;

    static constexpr double cellWidth = 10;
    static constexpr double cellHeight = 10; // Approximately sqrt(3)
    
    int rows;
    int cols;
    std::vector<std::vector<Grid::GlobalBin>> Ggrid;

    void calculateGridDimensions();
};

#endif // OPTICALROUTING_GRID_H
