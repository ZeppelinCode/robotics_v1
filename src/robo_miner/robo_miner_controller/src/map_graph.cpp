#include "robo_miner_controller/map_graph.h"
#include <iostream>
#include <sstream>

GraphNode::GraphNode(Coordinate coordinate, char blockType) : mCoordiante{coordinate}, mBlockType{blockType} {
}

void GraphNode::addNeighbour(std::shared_ptr<GraphNode> node) {
    mNeighbours.emplace_back(node);
}

Coordinate GraphNode::getCoordinate() {
    return mCoordiante;
}

char GraphNode::getBlockType() {
    return mBlockType;
}

std::string GraphNode::toStringMetadata() {
    std::stringstream representation;
    representation << "x: " << mCoordiante.x 
                   << ", y: " << mCoordiante.y
                   << ", c: " << mBlockType;
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