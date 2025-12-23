/**
 * @file board.h
 * @brief Board class definition
 */
#ifndef BOARD_H
#define BOARD_H

#include <vector>
#include <tuple>
#include <string>
#include <random>

struct Point
{
    double x;
    double y;
};

struct Component{
    std::string name;
    std::vector<Point> holes;
};

struct Pattern{
    std::string name;
    std::vector<Component> components;
};

class Board
{
public:
    Board(const int& board_size, const std::vector<Pattern>& patterns);
    bool isValid() const;
    int getSize() const;
    const std::vector<Point>& getHoles() const;

private:
    int board_size;
    std::vector<Pattern> patterns;
    bool valid;

    std::vector<Point> occupiedHoles;
    std::mt19937 rng;

    bool arrangePatterns(const Pattern& pattern);
    bool collide(const Point& a, const Point& b, double minDist = 1.0);
    std::pair<Point, Point> boundingBox(const Pattern& p);
};

#endif // BOARD_H
