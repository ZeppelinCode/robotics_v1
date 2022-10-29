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

namespace map_graph {
    constexpr auto COLLISION_COORDINATE = 'X';
    constexpr auto UNEXPLORED_COORDINATE = '#';
    constexpr auto CHARGING_STATION_COORDINATE = '@';
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
    GraphNode(Coordinate coordinate, unsigned char blockType);
    std::string toString();
    Coordinate getCoordinate();
    unsigned char getBlockType();
    void setBlockType(char blockType);
    void markVisited();
    void shiftCoordinateToTheRightBy(Coordinate shiftAmount);
private:
    Coordinate mCoordiante;
    unsigned char mBlockType; // # -> edge, X -> obstacle
};

class MapGraph {
public:
    MapGraph();
    std::shared_ptr<GraphNode> addNode(const std::shared_ptr<GraphNode> node);
    std::string toString();
    bool hasCoordinateBeenVisited(const Coordinate& coordinate) const;
    std::vector<Coordinate> getUnvisitedCoordinates(const std::vector<Coordinate>& coordinatesOfInterest) const;
    std::optional<std::shared_ptr<GraphNode>> getNodeAtCoordinate(const Coordinate& coordinate) const;
    const std::vector<std::shared_ptr<GraphNode>> getNodes() const;
    void shiftAllNodeCoordiantesToTheRightBy(Coordinate shiftAmount);
private:
    std::vector<std::shared_ptr<GraphNode>> mNodes{};
};
#endif
