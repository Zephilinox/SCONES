//STD
#include <iostream>
#include <thread>

//LIBS
#include <SCONES/NES.hpp>

int main()
{
    NES nes;

    while (true)
    {
        nes.tick();
        std::cout << "clock: " << nes.get_clock() << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds{ 16 });
    }

    return 0;
}