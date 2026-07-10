#include "GR.h"
#include "Net.h"
#include "Astar.h"
#include "Grid.h"
#include <vector>
#include <queue>
#include <cmath>
#include "optimizer.h"
#include <string>

template<typename T, typename Cmp = std::less<T>>
using RoutingQueue = std::priority_queue<T, std::vector<T>, Cmp>;

static std::string routeDirectionCode(const AStar::Vec2i& from, const AStar::Vec2i& to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    if (dx == -1 && dy == 0) return "1";
    if (dx == -1 && dy == 1) return "2";
    if (dx == 0 && dy == 1) return "3";
    if (dx == 1 && dy == 1) return "4";
    if (dx == 1 && dy == 0) return "5";
    if (dx == 1 && dy == -1) return "6";
    if (dx == 0 && dy == -1) return "7";
    if (dx == -1 && dy == -1) return "8";
    return "0";
}

static double routeSegmentLength(const std::string& directionCode) {
    if (directionCode == "1" || directionCode == "5") return WIDTH_LENGTH;
    if (directionCode == "3" || directionCode == "7") return HEIGHT_LENGTH;
    return DIAGONAL_LENGTH;
}

static void addVisitedStep(std::vector<AStar::Vec2i>& visited, const AStar::Vec2i& current, const AStar::Vec2i& next) {
    visited.push_back(next);
}

static void markPathOnMap(std::vector<std::vector<Grid::GlobalBin>>& map, const AStar::CoordinateList& path, std::vector<AStar::Vec2i>& visited) {
    if (path.empty()) return;
    if (path.size() == 1) {
        for (auto point : path) visited.push_back(point);
        return;
    }
    for (size_t i = 0; i < path.size() - 1; ++i) {
        AStar::Vec2i current = path[i];
        AStar::Vec2i next = path[i + 1];
        if (i == 0) visited.push_back(current);
        addVisitedStep(visited, current, next);

        std::string directionCode = routeDirectionCode(next, current);
        double single_length = routeSegmentLength(directionCode);
        if (i == 0) {
            map[current.y][current.x].length += single_length / 2;
            map[current.y][current.x].pattern["0" + directionCode]++;
        }
        map[next.y][next.x].length += single_length / 2;

        if (i + 2 < path.size()) {
            AStar::Vec2i afterNext = path[i + 2];
            std::string exitCode = routeDirectionCode(next, afterNext);
            map[next.y][next.x].length += routeSegmentLength(exitCode) / 2;
            if (std::stoi(directionCode) > std::stoi(exitCode)) {
                std::swap(directionCode, exitCode);
            }
            map[next.y][next.x].pattern[directionCode + exitCode]++;
        } else {
            map[next.y][next.x].pattern["0" + directionCode]++;
        }
    }
}

static void appendPathSegments(std::vector<AStar::SeedSegment>& segments, const AStar::CoordinateList& path, bool hard) {
    if (path.size() < 2) return;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        segments.push_back({path[i], path[i + 1], hard});
    }
}

static void applyHistoryCostsToMap(std::vector<std::vector<Grid::GlobalBin>>& map, const std::vector<std::pair<AStar::Vec2i, double>>& historyCosts) {
    int rows = static_cast<int>(map.size());
    int cols = rows ? static_cast<int>(map[0].size()) : 0;
    for (const auto& item : historyCosts) {
        const auto& point = item.first;
        if (point.y >= 0 && point.y < rows && point.x >= 0 && point.x < cols) {
            map[point.y][point.x].historyCost += item.second;
        }
    }
}

GlobalRouting::GlobalRouting(double totalWidth, double totalHeight, std::vector<Net> netList, AStar::HeuristicFunction heuristic, bool diagonalMov_enable, int direction, bool useInputOrder, bool blockPins, int targetGuard, std::vector<AStar::Vec2i> staticBlocks, double rudyWeight, bool blockRoutedPaths, bool reservePinStubs, std::vector<AStar::CoordinateList> seedPaths, bool congestionAware, double congestionBeta, double congestionT, double edgeCapacity, double overflowPenalty, double minBendRadiusGrid, std::vector<std::pair<AStar::Vec2i, double>> historyCosts, double historyPenalty, size_t hardSeedPathCount, double pathLoss, double crossingLoss, double bendLoss30, double bendLoss45, double bendLoss60, double bendLoss90)
    : totalWidth(totalWidth), totalHeight(totalHeight), grid(totalWidth, totalHeight), direction(direction), useInputOrder(useInputOrder), blockPins(blockPins), blockRoutedPaths(blockRoutedPaths), reservePinStubs(reservePinStubs), targetGuard(targetGuard), rudyWeight(rudyWeight), staticBlocks(staticBlocks), seedPaths(seedPaths), congestionAware(congestionAware), congestionBeta(congestionBeta), congestionT(congestionT), edgeCapacity(edgeCapacity), overflowPenalty(overflowPenalty), minBendRadiusGrid(minBendRadiusGrid), historyPenalty(historyPenalty), pathLoss(pathLoss), crossingLoss(crossingLoss), bendLoss30(bendLoss30), bendLoss45(bendLoss45), bendLoss60(bendLoss60), bendLoss90(bendLoss90), historyCosts(historyCosts), hardSeedPathCount(hardSeedPathCount) {
    double cost = 0;
    grid.getCentralPoints();
    std::vector<std::vector<Grid::GlobalBin>>& map = grid.getGgrid();
    applyHistoryCostsToMap(map, historyCosts);
    RoutingQueue<Net, Net_cmp> pq;

    for (size_t index = 0; index < seedPaths.size(); ++index) {
        bool hard = index < hardSeedPathCount;
        std::vector<AStar::Vec2i> ignoredVisited;
        markPathOnMap(map, seedPaths[index], hard ? visited : ignoredVisited);
        appendPathSegments(routedSeedSegments, seedPaths[index], hard);
    }

    for (int i = 0; i < netList.size(); ++i){
        pq.push(netList[i]);
    }

    std::vector<Net> nets;
    RoutingQueue<Net, Net_cmp> tmp = pq;
    while (!tmp.empty()) {
        nets.push_back(tmp.top());
        tmp.pop();
    }
    if (useInputOrder) {
        nets = netList;
    }

    for (Net net : nets){
        Point Gsource = net.getSource();
        Point Gtarget = net.getTarget();

        grid.setToCellCenterIdx(Gsource);
        grid.setToCellCenterIdx(Gtarget);
        net.setSource(Gsource);
        net.setTarget(Gtarget);

        auto res = SingleNetRouting(map, net, heuristic, diagonalMov_enable,nets);

        
        cost += res.second;
        routedNets.push_back({net.getName(), res.first, res.second});

    }

    totalCost = cost;
    
};

std::pair<AStar::CoordinateList, double> GlobalRouting::SingleNetRouting(std::vector<std::vector<Grid::GlobalBin>>& map, Net net, AStar::HeuristicFunction heuristic, bool diagonalMov_enable ,std::vector<Net> nets){
    Point source = net.getSource();
    Point target = net.getTarget();
    int row = static_cast<int>(map.size());
    int col = static_cast<int>(map[0].size());
    int sourceDirection = net.getSourceDirection() >= 0 ? net.getSourceDirection() : direction;
    int targetDirection = net.getTargetDirection();
    auto single_route_res = Astar_getPath(map, source, target, {col, row}, heuristic, diagonalMov_enable,nets, sourceDirection, targetDirection);
    auto path = single_route_res.first;
    double path_cost = single_route_res.second;
    double single_length = 0;
    if (path.size() <= 2) 
    {   
        if (!path.empty()) {
            visited.push_back(path[0]);
        }
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            addVisitedStep(visited, path[i], path[i + 1]);
        }
        appendPathSegments(routedSeedSegments, path, false);
        return {path, path_cost};
    }
    for (size_t i = 0; i < path.size() - 1; ++i) {
        AStar::Vec2i current = path[i];
        AStar::Vec2i next = path[i + 1];
        if (i==0) visited.push_back(current);
        addVisitedStep(visited, current, next);

        std::string directionCode = calculateDirectionCode(next, current);
        if (directionCode == "1" || directionCode == "5") {
            single_length = WIDTH_LENGTH;
        } 
        else if(directionCode == "3" || directionCode == "7"){
            single_length = HEIGHT_LENGTH;
        }
        else {
            single_length = DIAGONAL_LENGTH;
        }
        if (i == 0) {
            map[current.y][current.x].length += single_length/2;
            map[current.y][current.x].pattern["0" + directionCode]++;
        }      
        map[next.y][next.x].length += single_length/2;
        single_length = 0;

        if (i + 2 < path.size()) {
            AStar::Vec2i afterNext = path[i + 2];
            std::string exitCode = calculateDirectionCode(next, afterNext);
            if (exitCode == "1" || exitCode == "5") {
                single_length = WIDTH_LENGTH;
            } 
            else if(exitCode == "3" || exitCode == "7"){
                single_length = HEIGHT_LENGTH;
            }
            else {
                single_length = DIAGONAL_LENGTH;
            }
            map[next.y][next.x].length += single_length/2;
            if (std::stoi(directionCode) > std::stoi(exitCode)) {
                std::swap(directionCode, exitCode);
            }
            map[next.y][next.x].pattern[directionCode + exitCode]++;         
        }
        else{
            map[next.y][next.x].pattern["0" + directionCode]++;      
        }
    }
    appendPathSegments(routedSeedSegments, path, false);
    return {path, path_cost};
    }

std::pair<AStar::CoordinateList, double> GlobalRouting::Astar_getPath(std::vector<std::vector<Grid::GlobalBin>>& map, Point source, Point target, AStar::Vec2i worldSize, AStar::HeuristicFunction heuristic, bool diagonalMov_enable,std::vector<Net> nets, int sourceDirection, int targetDirection){
    AStar::Vec2i start = {
        static_cast<int>(source.getX()), static_cast<int>(source.getY())};
    AStar::Vec2i end = {
        static_cast<int>(target.getX()), static_cast<int>(target.getY())};
    std::vector<AStar::Vec2i> pinDirections = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}
    };
    AStar::Vec2i routeEnd = end;
    AStar::Vec2i targetStub = end;
    bool appendTarget = false;
    if (targetDirection >= 0 && targetDirection < static_cast<int>(pinDirections.size())) {
        targetStub = end + pinDirections[targetDirection];
        routeEnd = targetStub;
        appendTarget = true;
    }
    AStar::Vec2i startStub = start;
    if (sourceDirection >= 0 && sourceDirection < static_cast<int>(pinDirections.size())) {
        startStub = start + pinDirections[sourceDirection];
    }

    AStar::Generator generator;
    auto inBounds = [&](const AStar::Vec2i& cell) {
        return cell.x >= 0 && cell.x < worldSize.x && cell.y >= 0 && cell.y < worldSize.y;
    };
    auto sameCell = [](const AStar::Vec2i& a, const AStar::Vec2i& b) {
        return a.x == b.x && a.y == b.y;
    };
    auto addHardBlock = [&](const AStar::Vec2i& cell) {
        if (inBounds(cell) && !sameCell(cell, start) && !sameCell(cell, routeEnd) && !sameCell(cell, targetStub)) {
            generator.addCollision(cell);
        }
    };
    auto addPinBlock = [&](const AStar::Vec2i& cell) {
        if (inBounds(cell) && !sameCell(cell, start) && !sameCell(cell, routeEnd) && !sameCell(cell, startStub) && !sameCell(cell, targetStub)) {
            generator.addCollision(cell);
        }
    };
    for (const auto& block : staticBlocks) {
        addHardBlock(block);
    }
    if (blockRoutedPaths) {
        for (const auto& cell : visited) {
            addHardBlock(cell);
        }
    }

    if (blockPins || reservePinStubs) for (const Net& net : nets) {

        AStar::Vec2i start_0 = {
            static_cast<int>(net.getSource().getX()/10),
            static_cast<int>(net.getSource().getY()/10)
        };
        AStar::Vec2i end_0 = {
            static_cast<int>(net.getTarget().getX()/10),
            static_cast<int>(net.getTarget().getY()/10)
        };
        if (blockPins) {
            addPinBlock(start_0);
            addPinBlock(end_0);
        }
        if (reservePinStubs) {
            int sourceDir = net.getSourceDirection() >= 0 ? net.getSourceDirection() : direction;
            int targetDir = net.getTargetDirection();
            if (sourceDir >= 0 && sourceDir < static_cast<int>(pinDirections.size())) {
                addPinBlock(start_0 + pinDirections[sourceDir]);
            }
            if (targetDir >= 0 && targetDir < static_cast<int>(pinDirections.size())) {
                addPinBlock(end_0 + pinDirections[targetDir]);
            }
        }
    }

    if (targetGuard > 0) for (const Net& net : nets) {
        for (int offset = 1; offset <= targetGuard; ++offset) {
            AStar::Vec2i guard = {
                static_cast<int>(net.getTarget().getX() / 10) + offset,
                static_cast<int>(net.getTarget().getY() / 10)
            };
            generator.addCollision(guard);
        }
    }

    generator.setWorldSize(worldSize);
    generator.setDiagonalMovement(diagonalMov_enable);
    generator.setHeuristic(heuristic);
    generator.setRudyWeight(rudyWeight);
    generator.setCongestionAware(congestionAware, congestionBeta, congestionT, edgeCapacity, overflowPenalty);
    generator.setMinBendRadius(minBendRadiusGrid);
    generator.setHistoryPenalty(historyPenalty);
    generator.setLossModel(pathLoss, crossingLoss, bendLoss30, bendLoss45, bendLoss60, bendLoss90);
    generator.setSeedSegments(routedSeedSegments);

    auto route_res = generator.findPath(start, routeEnd, map, sourceDirection, -1);
    if (appendTarget && !route_res.first.empty()) {
        route_res.first.insert(route_res.first.begin(), end);
    }

    return route_res;    
}

std::string GlobalRouting::calculateDirectionCode(const AStar::Vec2i& from, const AStar::Vec2i& to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;

    std::string code;
    if (dx == -1 && dy == 0) code = "1";
    else if (dx == -1 && dy == 1) code = "2";
    else if (dx == 0 && dy == 1) code = "3";
    else if (dx == 1 && dy == 1) code = "4";
    else if (dx == 1 && dy == 0) code = "5";
    else if (dx == 1 && dy == -1) code = "6";
    else if (dx == 0 && dy == -1) code = "7";
    else if (dx == -1 && dy == -1) code = "8";
    return code;
}
