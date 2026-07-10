#ifndef OPTICALROUTING_GR_H
#define OPTICALROUTING_GR_H

#include "Astar.h"
#include "Net.h"
#include "Grid.h"
#include "optimizer.h"
#include <vector>

class GlobalRouting {
public:
    struct RoutedNet {
        std::string name;
        AStar::CoordinateList path;
        double cost = 0.0;
    };

    bool diagonalMov_enable = true;
    AStar::HeuristicFunction heuristic = AStar::Heuristic::octagonal;
    int direction;
    bool useInputOrder = false;
    bool blockPins = true;
    bool blockRoutedPaths = false;
    bool reservePinStubs = false;
    int targetGuard = 0;
    double rudyWeight = 0.0;
    std::vector<AStar::Vec2i> staticBlocks;
    std::vector<AStar::CoordinateList> seedPaths;
    bool congestionAware = false;
    double congestionBeta = 1.5;
    double congestionT = 1.6094379124341003;
    double edgeCapacity = 1.0;
    double overflowPenalty = 99999.0;
    double minBendRadiusGrid = 0.0;
    double historyPenalty = 0.0;
    double pathLoss = PATH_LOSS;
    double crossingLoss = CROSS_LOSS;
    double bendLoss30 = BEND_LOSS_30;
    double bendLoss45 = BEND_LOSS_45;
    double bendLoss60 = BEND_LOSS_60;
    double bendLoss90 = BEND_LOSS_90;
    std::vector<std::pair<AStar::Vec2i, double>> historyCosts;
    size_t hardSeedPathCount = 0;
    std::vector<AStar::SeedSegment> routedSeedSegments;
    std::vector<RoutedNet> routedNets;
    double totalCost = 0.0;

    std::vector<AStar::Vec2i> visited;

    struct Net_cmp{
        bool operator() (const Net& a, const Net& b){
            if (a.HPWL() != b.HPWL()){
                return a.HPWL() > b.HPWL();      
            } 
            else if (a.RipNum != b.RipNum){
                return a.RipNum < b.RipNum;
            }
            else {
                return false;
            }
        }
    };
    GlobalRouting(double totalWidth, double totalHeight, std::vector<Net> netList, AStar::HeuristicFunction heuristic, bool diagonalMov_enable, int direction, bool useInputOrder = false, bool blockPins = true, int targetGuard = 0, std::vector<AStar::Vec2i> staticBlocks = {}, double rudyWeight = 0.0, bool blockRoutedPaths = false, bool reservePinStubs = false, std::vector<AStar::CoordinateList> seedPaths = {}, bool congestionAware = false, double congestionBeta = 1.5, double congestionT = 1.6094379124341003, double edgeCapacity = 1.0, double overflowPenalty = 99999.0, double minBendRadiusGrid = 0.0, std::vector<std::pair<AStar::Vec2i, double>> historyCosts = {}, double historyPenalty = 0.0, size_t hardSeedPathCount = 0, double pathLoss = PATH_LOSS, double crossingLoss = CROSS_LOSS, double bendLoss30 = BEND_LOSS_30, double bendLoss45 = BEND_LOSS_45, double bendLoss60 = BEND_LOSS_60, double bendLoss90 = BEND_LOSS_90);
    std::pair<AStar::CoordinateList, double> Astar_getPath(std::vector<std::vector<Grid::GlobalBin>>& map, Point source, Point target, AStar::Vec2i worldSize, AStar::HeuristicFunction heuristic, bool diagonalMov_enable,std::vector<Net> nets, int sourceDirection, int targetDirection);
    void setHeuristic(AStar::HeuristicFunction heuristic){this->heuristic = heuristic;};
    void setDiagonalMov(bool diagonalMov_enable){this->diagonalMov_enable = diagonalMov_enable;};
    std::pair<AStar::CoordinateList, double> SingleNetRouting(std::vector<std::vector<Grid::GlobalBin>>& map, Net net, AStar::HeuristicFunction heuristic, bool diagonalMov_enable,std::vector<Net> nets);
    std::string calculateDirectionCode(const AStar::Vec2i& from, const AStar::Vec2i& to);

    Grid& getGrid()  { return grid; }
    const std::vector<RoutedNet>& getRoutedNets() const { return routedNets; }
    double getTotalCost() const { return totalCost; }

private:
    double totalWidth;
    double totalHeight;
    Grid grid;
};

#endif // OPTICALROUTING_GR_H
