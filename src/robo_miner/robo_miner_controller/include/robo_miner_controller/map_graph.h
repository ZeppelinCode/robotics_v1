#ifndef _H_MAP_GRAPH
#define _H_MAP_GRAPH

#include <vector>
#include <memory>
#include <string>
#include <string>
#include <optional>

// https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

struct Coordinate {
    ssize_t x;
    ssize_t y;

    Coordinate(ssize_t x, ssize_t y) : x{x}, y{y} {}

    std::string toString() const {
        return string_format("(%d, %d)", x, y);
    }
    bool operator==(const Coordinate& o)const {
        return o.x == x && o.y == y;
    }
};

class GraphNode {
public:
    GraphNode(Coordinate coordinate, char blockType);
    void addNeighbour(std::shared_ptr<GraphNode> node);
    std::string toString();
    std::string toStringMetadata();
    Coordinate getCoordinate();
    char getBlockType();
    bool hasBeenVisited();
    void markVisited();
    void shiftCoordinateToTheRightBy(Coordinate shiftAmount);
private:
    Coordinate mCoordiante;
    char mBlockType; // # -> edge, X -> obstacle
    std::vector<std::shared_ptr<GraphNode>> mNeighbours{};
    bool mHasBeenVisited;
};

class MapGraph {
public:
    MapGraph();
    void addNode(const std::shared_ptr<GraphNode> node);
    std::string toString();
    bool hasCoordinateBeenVisited(const Coordinate& coordinate) const;
    std::optional<std::shared_ptr<GraphNode>> getNodeAtCoordinate(const Coordinate& coordinate) const;
    const std::vector<std::shared_ptr<GraphNode>> getNodes() const;
    void shiftAllNodeCoordiantesToTheRightBy(Coordinate shiftAmount);
private:
    std::vector<std::shared_ptr<GraphNode>> mNodes{};
};
#endif
