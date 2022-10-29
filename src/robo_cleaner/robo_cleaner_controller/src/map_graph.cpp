#include "robo_cleaner_controller/map_graph.h"
#include <iostream>
#include <sstream>

GraphNode::GraphNode(Coordinate coordinate, unsigned char blockType) : mCoordiante{coordinate}, mBlockType{blockType} {
}

Coordinate GraphNode::getCoordinate() {
    return mCoordiante;
}

unsigned char GraphNode::getBlockType() {
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

std::vector<Coordinate> MapGraph::getUnvisitedCoordinates(const std::vector<Coordinate>& coordinatesOfInterest) const {
    std::vector<Coordinate> retval{};
    for (const auto &targetCoord: coordinatesOfInterest) {
        if (hasCoordinateBeenVisited(targetCoord)) {
            continue;
        }
        retval.emplace_back(targetCoord);
    }
    return retval;
}

std::optional<std::shared_ptr<GraphNode>> MapGraph::getNodeAtCoordinate(const Coordinate& targetCoordinate) const {
    for (const auto& node: mNodes) {
        auto nodeCoordinate = node->getCoordinate();
        if (nodeCoordinate.x == targetCoordinate.x && nodeCoordinate.y == targetCoordinate.y) {
            return node; 
        }
    }
    return std::nullopt;
}

const std::vector<std::shared_ptr<GraphNode>> MapGraph::getNodes() const {
    return mNodes;
}

std::string GraphNode::toString() {
    std::stringstream representation;
    representation << "x: " << mCoordiante.x 
                   << ", y: " << mCoordiante.y
                   << ", c: " << mBlockType;
    return representation.str();
}

MapGraph::MapGraph() {

}

void MapGraph::addNode(std::shared_ptr<GraphNode> newNode) {
    auto possiblyExistingNode = getNodeAtCoordinate(newNode->getCoordinate());
    if (possiblyExistingNode) {
        possiblyExistingNode->get()->setBlockType(newNode->getBlockType());
        return;
    }
    mNodes.emplace_back(newNode);
}

void MapGraph::shiftAllNodeCoordiantesToTheRightBy(Coordinate shiftAmount) {
    for (auto& node: mNodes) {
        node->shiftCoordinateToTheRightBy(shiftAmount);
    }
}

void GraphNode::shiftCoordinateToTheRightBy(Coordinate shiftAmount) {
    mCoordiante.x -= shiftAmount.x;
    mCoordiante.y -= shiftAmount.y;
}

void GraphNode::setBlockType(char type) {
    mBlockType = type;
}

std::string MapGraph::toString() {
    std::stringstream representation;
    for (const auto& node : mNodes) {
        representation << node->toString() << std::endl;
    }
    return representation.str();
}