#include "robo_miner_controller/map_graph.h"
#include <iostream>
#include <sstream>

GraphNode::GraphNode(Coordinate coordinate, char blockType) : mCoordiante{coordinate}, mBlockType{blockType}, mHasBeenVisited{false} {
}

void GraphNode::addNeighbour(std::shared_ptr<GraphNode> node) {
    mNeighbours.emplace_back(node);
}

bool GraphNode::hasBeenVisited() {
    return mHasBeenVisited;
}

void GraphNode::markVisited() {
    mHasBeenVisited = true;
}

Coordinate GraphNode::getCoordinate() {
    return mCoordiante;
}

char GraphNode::getBlockType() {
    return mBlockType;
}

bool MapGraph::hasCoordinateBeenVisited(const Coordinate& targetCoordinate) const {
    for (const auto& node: mNodes) {
        auto nodeCoordinate = node->getCoordinate();
        if (nodeCoordinate.x == targetCoordinate.x && nodeCoordinate.y == targetCoordinate.y) {
            return true; 
        }
    }
    return false;
}

std::string GraphNode::toStringMetadata() {
    std::stringstream representation;
    representation << "x: " << mCoordiante.x 
                   << ", y: " << mCoordiante.y
                   << ", c: " << mBlockType
                   << ", visited: " << mHasBeenVisited;
    return representation.str();
}

std::string GraphNode::toString() {
    std::stringstream representation;
    representation << toStringMetadata() << ", neighbours: (";
    for (const auto& neighbour: mNeighbours) {
        representation << neighbour->toStringMetadata() << ',';
    }
    representation << ")";
    return representation.str();
}

MapGraph::MapGraph() {

}

void MapGraph::addNode(std::shared_ptr<GraphNode> node) {
    mNodes.emplace_back(node);
}

std::string MapGraph::toString() {
    std::stringstream representation;
    for (const auto& node : mNodes) {
        representation << node->toString() << std::endl;
    }
    return representation.str();
}