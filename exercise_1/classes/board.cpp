#include "board.h"
#include <cmath>
#include <iostream>
#include <algorithm>

Board::Board(const int& board_size, const std::vector<Pattern>& patterns)
    : board_size(board_size),
      patterns(patterns),
      rng(std::random_device{}()),
      valid(true)
{
    for (const auto& pattern : patterns)
    {
        if (!arrangePatterns(pattern))
        {
            std::cout << "Failed to arrange all patterns on the board.\n";
            valid = false;
            break;
        }
    }
}

bool Board::isValid() const
{
    return valid;
}

int Board::getSize() const
{
    return board_size;
}

const std::vector<Point>& Board::getHoles() const
{
    return occupiedHoles;
}

bool Board::collide(const Point& a, const Point& b, double minDist)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy) < minDist;
}

std::pair<Point, Point> Board::boundingBox(const Pattern& p)
{
    Point min{1e9, 1e9}, max{-1e9, -1e9};

    for (const auto& c : p.components)
        for (const auto& h : c.holes)
        {
            min.x = std::min(min.x, h.x);
            min.y = std::min(min.y, h.y);
            max.x = std::max(max.x, h.x);
            max.y = std::max(max.y, h.y);
        }

    return {min, max};
}

bool Board::arrangePatterns(const Pattern& pattern)
{
    auto [minBB, maxBB] = boundingBox(pattern);
    double width  = maxBB.x - minBB.x;
    double height = maxBB.y - minBB.y;

    std::uniform_real_distribution<double> distX(0, board_size - width);
    std::uniform_real_distribution<double> distY(0, board_size - height);

    constexpr int MAX_TRIES = 500;

    for (int attempt = 0; attempt < MAX_TRIES; ++attempt)
    {
        double offsetX = distX(rng) - minBB.x;
        double offsetY = distY(rng) - minBB.y;

        std::vector<Point> translated;
        bool collisionDetected = false;

        for (const auto& comp : pattern.components)
        {
            for (const auto& hole : comp.holes)
            {
                Point p{hole.x + offsetX, hole.y + offsetY};

                if (p.x < 0 || p.y < 0 ||
                    p.x > board_size || p.y > board_size)
                {
                    collisionDetected = true;
                    break;
                }

                for (const auto& o : occupiedHoles)
                {
                    if (collide(p, o))
                    {
                        collisionDetected = true;
                        break;
                    }
                }

                if (collisionDetected) break;
                translated.push_back(p);
            }
            if (collisionDetected) break;
        }

        if (!collisionDetected)
        {
            occupiedHoles.insert(
                occupiedHoles.end(),
                translated.begin(),
                translated.end()
            );

            std::cout << "Placed pattern: " << pattern.name << "\n";
            return true;
        }
    }

    std::cout << "Failed to place pattern: " << pattern.name << "\n";
    return false;
}
