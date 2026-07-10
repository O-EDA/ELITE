#ifndef OPTICALROUTING_ASTAR_H
#define OPTICALROUTING_ASTAR_H

#include <vector>
#include <functional>
#include <set>
#include "Grid.h"
#include <iostream>
#define WIDTH_LENGTH 1
#define HEIGHT_LENGTH 1
#define DIAGONAL_LENGTH 2

#define PATH_LOSS 1.5e-4 //um
#define BEND_LOSS_30 0.01
#define BEND_LOSS_45 0.015
#define BEND_LOSS_60 0.02
#define BEND_LOSS_90 0.03
#define CROSS_LOSS 0.52
#define CROSS_BEND_VIO 1
#define CROSS_PATTERN_VIO 2
#define CAPACITY_H 1
#define CAPACITY_W 1
#define CAPACITY_D 1

namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);        
        friend Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
            return{ left_.x + right_.x, left_.y + right_.y };
        }
        friend Vec2i operator - (const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
            return{ left_.x - right_.x, left_.y - right_.y };
        }
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<double(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct SeedSegment
    {
        Vec2i a, b;
        bool hard;
    };

    struct Node
    {
        double G, H;
        Vec2i coordinates;
        Node *parent;
        int num_crossing = 0;
        int turnCount = 0;
        double runLength = 0;

        Node(Vec2i coord_, Node *parent_ = nullptr);
        double getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        void setRudyWeight(double weight_);
        void setCongestionAware(bool enable_, double beta_ = 1.5, double t_ = 1.6094379124341003, double edgeCapacity_ = 1.0, double overflowPenalty_ = 99999.0);
        void setMinBendRadius(double radius_);
        void setHistoryPenalty(double penalty_);
        void setLossModel(double pathLoss_, double crossingLoss_, double bendLoss30_, double bendLoss45_, double bendLoss60_, double bendLoss90_);
        void setSeedSegments(std::vector<SeedSegment> seedSegments_);
        std::pair<CoordinateList,double> findPath(Vec2i source_, Vec2i target_, std::vector<std::vector<Grid::GlobalBin>>& map, int direction_source, int direction_target = -1);
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();
        uint detectCrossing(std::vector<std::vector<Grid::GlobalBin>>& map, std::string pattern, Vec2i coordinates);
        uint detectviolate(std::vector<std::vector<Grid::GlobalBin>>& map, std::string pattern, Vec2i coordinates);
        std::string calculateDirectionCode(const AStar::Vec2i& from, const AStar::Vec2i& to);
        double RUDY(std::vector<std::vector<Grid::GlobalBin>>& map, Vec2i source, Vec2i target);
        int calculateBend(std::string pattern);
        double calculateCost(Grid::GlobalBin bin, std::string directionCodeOut, double h, double k);
        double calculateCongestionPenalty(Grid::GlobalBin bin, std::string directionCodeOut);
        double calculateHistoryPenalty(Grid::GlobalBin bin);
        double calculateBendRadiusFactor(double runLength, double requiredRadius);
        double bendLossForAngle(int angle) const;
        bool violatesSeedSegments(Vec2i from, Vec2i to, uint& legalCrossings);

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
        uint directions;
        double rudyWeight;
        bool congestionAware = false;
        double congestionBeta = 1.5;
        double congestionT = 1.6094379124341003;
        double edgeCapacity = 1.0;
        double overflowPenalty = 99999.0;
        double minBendRadius = 0.0;
        double historyPenalty = 0.0;
        double pathLoss = PATH_LOSS;
        double crossingLoss = CROSS_LOSS;
        double bendLoss30 = BEND_LOSS_30;
        double bendLoss45 = BEND_LOSS_45;
        double bendLoss60 = BEND_LOSS_60;
        double bendLoss90 = BEND_LOSS_90;
        std::vector<SeedSegment> seedSegments;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static double octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // OPTICALROUTING_ASTAR_H
