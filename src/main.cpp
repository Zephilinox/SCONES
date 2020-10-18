//STD
#include <iostream>
#include "NES/NES.hpp"

int main()
{
    NES nes;
    std::cout << "hello\n";

    for (int i = 0; i < 1000; i++)
    {
        nes.tick();
        std::cout << "ticking";
    }

    return 0;
}