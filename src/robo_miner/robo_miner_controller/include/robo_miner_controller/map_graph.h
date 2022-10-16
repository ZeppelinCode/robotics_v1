#ifndef _H_MAP_GRAPH
#define _H_MAP_GRAPH

#include <vector>
#include <memory>
#include <string>

struct Coordinate {
    ssize_t x;
    ssize_t y;

    Coordinate(ssize_t x, ssize_t y) : x{x}, y{y} {}
};

class GraphNode {
public:
    GraphNode(Coordinate coordinate, char blockType);
    void addNeighbour(std::shared_ptr<GraphNode> node);
    std::string toString();
    std::string toStringMetadata();
    Coordinate getCoordinate();
    char getBlockType();
private:
    Coordinate mCoordiante;
    char mBlockType; // # -> edge, X -> obstacle
    std::vector<std::shared_ptr<GraphNode>> mNeighbours{};
};

class MapGraph {
public:
    MapGraph();
    void addNode(std::shared_ptr<GraphNode> node);
    std::string toString();
private:
    std::vector<std::shared_ptr<GraphNode>> mNodes{};
};
#endif
