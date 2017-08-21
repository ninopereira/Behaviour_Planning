#include <vector>
#include <iostream>

int main()
{
    using VecInt = std::vector<int>;
    using VecList = std::vector<VecInt>;

    VecInt foo = {1,2,3,4,5};
    VecInt bar = foo;
    VecList list1 = {foo, bar};
    VecList list2 = list1;
    bar.erase(bar.begin());
    list1[1].erase(list1[1].begin());
    std::cout << "Printing list 1" << std::endl;
    for (auto it_vec: list1)
    {
        std::cout << "vector contains: ";
        for (int i:it_vec)
        {
            std::cout << i << ", ";
        }
        std::cout << std::endl;
    }

    std::cout << "Printing list 2" << std::endl;
    for (auto it_vec: list2)
    {
        std::cout << "vector contains: ";
        for (int i:it_vec)
        {
            std::cout << i << ", ";
        }
        std::cout << std::endl;
    }
}
