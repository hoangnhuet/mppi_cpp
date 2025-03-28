#include<vector>
class Costmap2D
{
public:
    Costmap2D(int width, int height)
        : costmap(height, std::vector<int>(width, 0))
    {}

    int getCost(int x, int y) const
    {
        return costmap[y][x];
    }

    void setCost(int x, int y, int cost)
    {
        costmap[y][x] = cost;
    }

private:
    std::vector<std::vector<int>> costmap;
};