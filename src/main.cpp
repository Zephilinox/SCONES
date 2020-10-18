//STD
#include <iostream>
#include "NES/NES.hpp"

int main()
{
    NES nes;
    nes.tick();
    std::cout << "hello\n";
    return 0;
}