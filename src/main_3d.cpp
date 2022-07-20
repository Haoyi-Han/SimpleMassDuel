#include <iostream>
#include <cmath>
#include <vector>
#include <xtensor.hpp>

int main(int argc, char* argv[])
{
    xt::xarray<double> arr1
    { {1.0, 2.0, 3.0},
     {2.0, 5.0, 7.0},
     {2.0, 5.0, 7.0} };

    xt::xarray<double> arr2
    { 5.0, 6.0, 7.0 };

    xt::xarray<double> res = xt::view(arr1, 0) + arr2;

    std::vector<double> vec1(5, 1.0);
    auto l(std::ssize(vec1));

    std::cout << sizeof(l) << std::endl;

    std::cout << res << std::endl;

    return 0;
}