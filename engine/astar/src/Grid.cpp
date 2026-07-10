#include "Grid.h"
#include <cmath> // For floor and ceil

Grid::Grid(double totalWidth, double totalHeight)
    : totalWidth(totalWidth), totalHeight(totalHeight) {
    calculateGridDimensions();
    Ggrid.resize(rows);
    for (auto &row : Ggrid) {
        row.resize(cols);
    }
}

void Grid::calculateGridDimensions() {
    cols = static_cast<int>(ceil(totalWidth / cellWidth+1));
    rows = static_cast<int>(ceil(totalHeight / cellHeight+1));
}

void Grid::getCentralPoints(){
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            double centerX = (col + 0.5) * cellWidth;
            double centerY = (row + 0.5) * cellHeight;
            Ggrid[row][col] = GlobalBin(centerX, centerY);
        }
    }
}

void Grid::setToCellCenter(Point& point) const {
    int col = static_cast<int>(floor(point.getX() / cellWidth));
    int row = static_cast<int>(floor(point.getY() / cellHeight));

    col = std::max(0, std::min(col, cols - 1));
    row = std::max(0, std::min(row, rows - 1));

    double centerX = (col + 0.5) * cellWidth;
    double centerY = (row + 0.5) * cellHeight;

    point.setX(centerX);
    point.setY(centerY);
}

void Grid::setToCellCenterIdx(Point& point) const {
    int col = static_cast<int>(floor(point.getX() / cellWidth));
    int row = static_cast<int>(floor(point.getY() / cellHeight));
    
    col = std::max(0, col);
    row = std::max(0, row);

    point.setX(col);
    point.setY(row);
}
