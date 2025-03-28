#include<xtensor/xtensor.hpp>
#include<xtensor/xview.hpp>
#include<iostream>
namespace test_xt
{
    struct test_xtensor
    {
        xt::xtensor<float,2> x;
        xt::xtensor<float,2> y;
        xt::xtensor<float,2> yaw;
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        x = xt::zeros<float>({batch_size, time_steps});
        y = xt::zeros<float>({batch_size, time_steps});
        yaw = xt::zeros<float>({batch_size, time_steps});
    }
    };
};
int main()
{
    test_xt::test_xtensor test;
    test.reset(10, 10);
    std::cout<<test.x(2,3)<<std::endl;
    double a = 19;
    std::cout<<a - test.x(2,3)<<std::endl;
    // get size of xt
    std::cout<<sizeof(test.x)<<std::endl;
    test.reset(20, 10);
    std::cout<<sizeof(test.x)<<std::endl;
    test.x(2,3) = 10;
    test.x(2,4) = 20;
    test.x(1,3) = 10;
    test.x(1,4) = 20;
    test.x(3,3) = 10;
    test.x(3,4) = 20;
    test.x(4,3) = 10;
    test.x(4,4) = 20;
    test.x(6,3) = 10;
    test.x(9,4) = 20;
    test.x(2,9) = 10;
    test.x(4,1) = 20;
    test.x(2,3) = 10;
    test.x(2,4) = 20;
    test.x(1,3) = 10;
    test.x(1,4) = 20;
    test.x(3,9) = 10;
    test.x(3,7) = 20;
    test.x(4,6) = 10;
    test.x(9,5) = 20;
    test.x(9,3) = 10;
    test.x(9,4) = 20;
    test.x(9,9) = 10;
    test.x(9,1) = 20;
    std::cout<<sizeof(a)<<std::endl;
}