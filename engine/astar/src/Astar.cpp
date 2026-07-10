#include "Astar.h"
#include <string>
#include <algorithm>
#include <math.h>
#include <map>
#include <unordered_map>
#include <array>

using namespace std::placeholders;

using uint = unsigned int;

static std::vector<std::pair<int, double>> demandComponentsForDirection(int direction)
{
    switch (direction) {
        case 1: return {{1, 1.0}};
        case 3: return {{3, 1.0}};
        case 5: return {{5, 1.0}};
        case 7: return {{7, 1.0}};
        case 2:
        case 4:
        case 6:
        case 8:
            return {{1, 0.25}, {3, 0.25}, {5, 0.25}, {7, 0.25}};
        default: return {};
    }
}

static std::array<double, 9> edgeDemandFromPatterns(const Grid::GlobalBin& bin)
{
    std::array<double, 9> demand{};
    for (const auto& entry : bin.pattern) {
        if (entry.second == 0 || entry.first.size() < 2) {
            continue;
        }
        int a = entry.first[0] - '0';
        int b = entry.first[1] - '0';
        for (const auto& component : demandComponentsForDirection(a)) {
            demand[component.first] += component.second * entry.second;
        }
        for (const auto& component : demandComponentsForDirection(b)) {
            demand[component.first] += component.second * entry.second;
        }
    }
    return demand;
}

static std::string normalizedPattern(std::string pattern)
{
    if (pattern.size() < 2) return pattern;
    if (pattern[0] > pattern[1]) std::swap(pattern[0], pattern[1]);
    return pattern.substr(0, 2);
}

static std::string straightPatternKind(const std::string& pattern)
{
    std::string key = normalizedPattern(pattern);
    if (key == "15") return "horizontal";
    if (key == "37") return "vertical";
    if (key == "26") return "diag_neg";
    if (key == "48") return "diag_pos";
    return "";
}

static bool legalPatternCross(const std::string& left, const std::string& right)
{
    std::string a = straightPatternKind(left);
    std::string b = straightPatternKind(right);
    if (a.empty() || b.empty()) return false;
    return ((a == "horizontal" && b == "vertical") ||
            (a == "vertical" && b == "horizontal") ||
            (a == "diag_pos" && b == "diag_neg") ||
            (a == "diag_neg" && b == "diag_pos"));
}

static std::string segmentKind(const AStar::Vec2i& a, const AStar::Vec2i& b)
{
    int dx = b.x - a.x;
    int dy = b.y - a.y;
    if (dy == 0) return "horizontal";
    if (dx == 0) return "vertical";
    if (std::abs(dx) == std::abs(dy)) return (dx == dy) ? "diag_pos" : "diag_neg";
    return "";
}

static bool legalSegmentCross(const std::string& left, const std::string& right)
{
    return ((left == "horizontal" && right == "vertical") ||
            (left == "vertical" && right == "horizontal") ||
            (left == "diag_pos" && right == "diag_neg") ||
            (left == "diag_neg" && right == "diag_pos"));
}

static int orientation(const AStar::Vec2i& a, const AStar::Vec2i& b, const AStar::Vec2i& c)
{
    int value = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

static bool inBox(const AStar::Vec2i& p, const AStar::Vec2i& a, const AStar::Vec2i& b)
{
    return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) &&
           std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}

static bool samePoint(const AStar::Vec2i& a, const AStar::Vec2i& b)
{
    return a.x == b.x && a.y == b.y;
}

static bool sharesEndpoint(const AStar::Vec2i& a, const AStar::Vec2i& b, const AStar::Vec2i& c, const AStar::Vec2i& d)
{
    return samePoint(a, c) || samePoint(a, d) || samePoint(b, c) || samePoint(b, d);
}

static bool sameUnitSegment(const AStar::Vec2i& a, const AStar::Vec2i& b, const AStar::Vec2i& c, const AStar::Vec2i& d)
{
    return ((samePoint(a, c) && samePoint(b, d)) || (samePoint(a, d) && samePoint(b, c)));
}

static bool segmentsIntersect(const AStar::Vec2i& a, const AStar::Vec2i& b, const AStar::Vec2i& c, const AStar::Vec2i& d)
{
    int o1 = orientation(a, b, c);
    int o2 = orientation(a, b, d);
    int o3 = orientation(c, d, a);
    int o4 = orientation(c, d, b);
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && inBox(c, a, b)) return true;
    if (o2 == 0 && inBox(d, a, b)) return true;
    if (o3 == 0 && inBox(a, c, d)) return true;
    if (o4 == 0 && inBox(b, c, d)) return true;
    return false;
}

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
    turnCount = parent_ ? parent_->turnCount : 0;
    runLength = parent_ ? parent_->runLength : 0;
}

double AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    rudyWeight = 0.0;
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::setRudyWeight(double weight_)
{
    rudyWeight = weight_;
}

void AStar::Generator::setCongestionAware(bool enable_, double beta_, double t_, double edgeCapacity_, double overflowPenalty_)
{
    congestionAware = enable_;
    congestionBeta = beta_;
    congestionT = t_;
    edgeCapacity = edgeCapacity_;
    overflowPenalty = overflowPenalty_;
}

void AStar::Generator::setMinBendRadius(double radius_)
{
    minBendRadius = radius_;
}

void AStar::Generator::setHistoryPenalty(double penalty_)
{
    historyPenalty = penalty_;
}

void AStar::Generator::setLossModel(double pathLoss_, double crossingLoss_, double bendLoss30_, double bendLoss45_, double bendLoss60_, double bendLoss90_)
{
    pathLoss = pathLoss_;
    crossingLoss = crossingLoss_;
    bendLoss30 = bendLoss30_;
    bendLoss45 = bendLoss45_;
    bendLoss60 = bendLoss60_;
    bendLoss90 = bendLoss90_;
}

void AStar::Generator::setSeedSegments(std::vector<SeedSegment> seedSegments_)
{
    seedSegments = std::move(seedSegments_);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

bool AStar::Generator::violatesSeedSegments(Vec2i from, Vec2i to, uint& legalCrossings)
{
    std::string candidateKind = segmentKind(from, to);
    for (const auto& segment : seedSegments) {
        if (!segmentsIntersect(from, to, segment.a, segment.b)) {
            continue;
        }
        if (sameUnitSegment(from, to, segment.a, segment.b)) {
            return true;
        }
        if (segment.hard) {
            return true;
        }
        if (sharesEndpoint(from, to, segment.a, segment.b)) {
            continue;
        }
        std::string existingKind = segmentKind(segment.a, segment.b);
        if (legalSegmentCross(candidateKind, existingKind)) {
            legalCrossings++;
            continue;
        }
        return true;
    }
    return false;
}

std::pair<AStar::CoordinateList, double> AStar::Generator::findPath(Vec2i source_, Vec2i target_, std::vector<std::vector<Grid::GlobalBin>>& map, int direction_source, int direction_target)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    bool foundTarget = false;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));
    std::vector<Vec2i> direction60 = { {-1, -1}, {-1, 0}, {-1, 1}, {1, 1}, {1, 0}, {1, -1} };     
    std::vector<Vec2i> direction30 = { {0, 1}, {1, 1}, {1, 0}, {0, -1}, {-1, -1}, {-1, 0} }; 

    auto validDirection = [&](int index) {
        return index >= 0 && index < static_cast<int>(directions);
    };

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        } 

        if (current->coordinates == target_ 
        ) {
            foundTarget = true;
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        if (current->coordinates == source_ && validDirection(direction_source)) {
            Vec2i newCoordinates(current->coordinates + direction[direction_source]);
            uint seed_cross = 0;
            if (!detectCollision(newCoordinates) && !violatesSeedSegments(current->coordinates, newCoordinates, seed_cross)) {
                Node *successor;
                successor = new Node(newCoordinates, current);
                successor->G = 0;
                double heuristicScale = PATH_LOSS > 0.0 ? pathLoss / PATH_LOSS : 1.0;
                successor->H = heuristic(successor->coordinates, target_) * heuristicScale;
                successor->turnCount = 0;
                successor->runLength = 1;
                openSet.push_back(successor);
            }
            continue;
        }

        for (uint i = 0; i < directions; ++i) {

            Vec2i newCoordinates(current->coordinates + direction[i]);
            uint seed_cross = 0;
            if (violatesSeedSegments(current->coordinates, newCoordinates, seed_cross)) {
                continue;
            }
            if (newCoordinates == target_ && validDirection(direction_target)) {
                Vec2i requiredTargetStub = target_ + direction[direction_target];
                if (!(current->coordinates == requiredTargetStub)) {
                    continue;
                }
            }
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }
            double congest_cost = 0;
            double congestion_penalty = 0;
            std::string out;
            out = calculateDirectionCode(newCoordinates, current->coordinates);
            if (congestionAware) {
                congestion_penalty = calculateCongestionPenalty(map[newCoordinates.y][newCoordinates.x], out);
                if (congestion_penalty >= overflowPenalty) {
                    continue;
                }
            } else {
                congest_cost = calculateCost(map[newCoordinates.y][newCoordinates.x], out, 0.02, 1);
            }
            double leng = ((i < 4) ? (i % 2 == 0) ? HEIGHT_LENGTH : WIDTH_LENGTH : DIAGONAL_LENGTH);
            uint n_cross = seed_cross;
            uint cross_violate = 0;        
            bool bend_flag = false;
            double bend_loss = 0;
            double bend_radius_factor = 1.0;
            double nextRunLength = current->runLength + leng;
            int nextTurnCount = current->turnCount;
          
            if (current->parent != nullptr) {
                Vec2i ori = current->coordinates - current->parent->coordinates;
                Vec2i new_dir = newCoordinates - current->coordinates;
                std::string directionCodeIn, directionCodeOut, pattern;
                directionCodeIn = calculateDirectionCode(current->coordinates, newCoordinates);
                directionCodeOut = calculateDirectionCode(current->coordinates, current->parent->coordinates);
                pattern = directionCodeIn + directionCodeOut;
                n_cross = detectCrossing(map, pattern, current->coordinates); 
                cross_violate = detectviolate(map, pattern, current->coordinates);
                if (cross_violate != 0) {
                    continue;
                }
                

                int angle;
                angle = calculateBend(pattern);
                if (angle > 90) {
                    continue;
                }
                if (angle) {
                    bend_flag = true;
                    double requiredRadius = (current->turnCount == 0) ? minBendRadius : 2.0 * minBendRadius;
                    bend_radius_factor = calculateBendRadiusFactor(current->runLength, requiredRadius);
                    if (bend_radius_factor >= overflowPenalty) {
                        continue;
                    }
                    nextRunLength = leng;
                    nextTurnCount = current->turnCount + 1;
                    bend_loss = bendLossForAngle(angle);
                } 
                else {
                    bend_flag = false;
                }
            }

            double penalty = 1.0;

            int total_cross;
            double stepCost = bend_radius_factor * (leng * pathLoss * penalty + bend_flag * bend_loss + n_cross * crossingLoss);
            double totalCost;
            if (congestionAware) {
                double history_penalty = calculateHistoryPenalty(map[newCoordinates.y][newCoordinates.x]);
                totalCost = current->G + (1.0 + congestion_penalty) * (1.0 + history_penalty) * stepCost;
            } else {
                totalCost = current->G + stepCost + congest_cost;
            }
            total_cross = current->num_crossing + n_cross;
            double Heu = heuristic(newCoordinates, target_);
            double ru =  RUDY(map, newCoordinates, target_);
            double peanalty_bend = 0;
            if ((newCoordinates.x - current->coordinates.x) * (target_.x - newCoordinates.x) >= 0 && 
            (newCoordinates.y - current->coordinates.y) * (target_.y - newCoordinates.y) >= 0) {
                peanalty_bend = bendLoss30;
            }

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                double heuristicScale = PATH_LOSS > 0.0 ? pathLoss / PATH_LOSS : 1.0;
                successor->H = heuristic(successor->coordinates, target_) * heuristicScale
                + RUDY(map, successor->coordinates, target_)
                ;
                successor->num_crossing = total_cross;
                successor->runLength = nextRunLength;
                successor->turnCount = nextTurnCount;
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
                successor->num_crossing = total_cross;
                successor->runLength = nextRunLength;
                successor->turnCount = nextTurnCount;
            }
        }
    }
    if (!foundTarget) {
        releaseNodes(openSet);
        releaseNodes(closedSet);
        return {CoordinateList(), 0.0};
    }

    double total_loss = current->G;
    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }
    releaseNodes(openSet);
    releaseNodes(closedSet);

    return {path, total_loss};
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

double AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    
    double loss_path = PATH_LOSS * (WIDTH_LENGTH * delta.x + HEIGHT_LENGTH * delta.y);
    return loss_path;
    
}

uint AStar::Generator::detectCrossing(std::vector<std::vector<Grid::GlobalBin>>& map, std::string pattern, AStar::Vec2i coordinates)
{
    uint n_cross = 0;
    const auto& cell = map[coordinates.y][coordinates.x].pattern;
    for (const auto& entry : cell) {
        if (entry.second == 0) {
            continue;
        }
        if (legalPatternCross(pattern, entry.first)) {
            n_cross += entry.second;
        }
    }
    return n_cross;
}

uint AStar::Generator::detectviolate(std::vector<std::vector<Grid::GlobalBin>>& map, std::string pattern, AStar::Vec2i coordinates)
{
    const auto& cell = map[coordinates.y][coordinates.x].pattern;
    for (const auto& entry : cell) {
        if (entry.second == 0) {
            continue;
        }
        if (!legalPatternCross(pattern, entry.first)) {
            return CROSS_PATTERN_VIO;
        }
    }
    return 0;
}

std::string AStar::Generator::calculateDirectionCode(const AStar::Vec2i& from, const AStar::Vec2i& to) {
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

double AStar::Generator::RUDY(std::vector<std::vector<Grid::GlobalBin>>& map, AStar::Vec2i source, AStar::Vec2i target) {
    double rudy = 0.0;
    int bbox_area;
    bbox_area = abs(target.x - source.x) * abs(target.y - source.y);
    int maxX = std::max(source.x, target.x);
    int maxY = std::max(source.y, target.y);
    int minX = std::min(source.x, target.x);
    int minY = std::min(source.y, target.y);
    for (int i = minX; i <= maxX; ++i) {
        for (int j = minY; j <= maxY; ++j) {
            rudy += map[j][i].length;
        }
    }
    if (bbox_area == 0) return 0;

    else return rudy / bbox_area * rudyWeight;

}

int AStar::Generator::calculateBend(std::string pattern) {
    std::unordered_map<char, int> directionAngles = {
        {'1', 180}, // To the left (180 degrees from the positive x-axis)
        {'2', 120}, // Diagonal up-left (120 degrees from the positive x-axis)
        {'3', 90},  // Upwards (90 degrees from the positive x-axis)
        {'4', 60},  // Diagonal up-right (60 degrees from the positive x-axis)
        {'5', 0},   // To the right (0 degrees from the positive x-axis)
        {'6', 300}, // Diagonal down-right (300 degrees from the positive x-axis)
        {'7', 270}, // Downwards (270 degrees from the positive x-axis)
        {'8', 240}  // Diagonal down-left (240 degrees from the positive x-axis)
    };

    char inputDir = pattern[0];
    char outputDir = pattern[1];

    int inputAngle = directionAngles[inputDir];
    int outputAngle = directionAngles[outputDir];

    int bendAngle = std::abs(outputAngle - inputAngle);

    bendAngle = bendAngle % 360;

    if (bendAngle > 180) {
        bendAngle = 360 - bendAngle;
    }

    return 180 - bendAngle;
}

double AStar::Generator::calculateCost(Grid::GlobalBin bin, std::string directionCodeOut, double h, double k)
{
    int out = directionCodeOut[0] - '0';
    int demand = 1;
    for (int i = out + 1; i < 9; i++)
    {
        demand += bin.pattern[std::to_string(out) + std::to_string(i)];
    }
    for (int i = 0; i < out; i++)
    {
        demand += bin.pattern[std::to_string(i) + std::to_string(out)];
    }
    int capacity;
    if (out == 1 || out == 5) capacity = CAPACITY_H;
    else if (out == 3 || out == 7) capacity = CAPACITY_W;
    else capacity = CAPACITY_D;

    double exponent = -k * (demand - capacity);
    double cost = (h / (1 + std::exp(exponent)));

    if (demand > 1) cost = 99999;
    else if (demand <= 1) cost = 0;
    return cost;
}

double AStar::Generator::calculateCongestionPenalty(Grid::GlobalBin bin, std::string directionCodeOut)
{
    int out = directionCodeOut[0] - '0';
    double capacity = edgeCapacity;
    if (capacity <= 0.0) {
        capacity = 1.0;
    }
    auto existingDemand = edgeDemandFromPatterns(bin);
    auto candidateDemand = demandComponentsForDirection(out);
    if (candidateDemand.empty()) {
        return 0.0;
    }
    double weightedPenalty = 0.0;
    double totalWeight = 0.0;
    for (const auto& component : candidateDemand) {
        double demandAfterRoute = existingDemand[component.first] + component.second;
        double omega = (capacity - demandAfterRoute) / capacity;
        if (omega < -1e-9) {
            return overflowPenalty;
        }
        double edgePenalty = congestionBeta / (1.0 + std::exp(congestionT * omega)) * demandAfterRoute;
        weightedPenalty += component.second * edgePenalty;
        totalWeight += component.second;
    }
    return totalWeight > 0.0 ? weightedPenalty / totalWeight : 0.0;
}

double AStar::Generator::calculateHistoryPenalty(Grid::GlobalBin bin)
{
    if (historyPenalty <= 0.0 || bin.historyCost <= 0.0) {
        return 0.0;
    }
    return bin.historyCost * historyPenalty;
}

double AStar::Generator::calculateBendRadiusFactor(double runLength, double requiredRadius)
{
    if (minBendRadius <= 0.0 || requiredRadius <= 0.0) {
        return 1.0;
    }
    return runLength + 1e-9 >= requiredRadius ? 1.0 : overflowPenalty;
}

double AStar::Generator::bendLossForAngle(int angle) const
{
    if (angle == 30) return bendLoss30;
    if (angle == 45) return bendLoss45;
    if (angle == 60) return bendLoss60;
    if (angle == 90) return bendLoss90;
    return 0.0;
}
