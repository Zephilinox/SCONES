//STD
#include <ctime>

//LIBS
#include <SDL.h>

//SELF
#include "NotAGame.hpp"

int main(int, char**)
{
    std::srand(std::time(nullptr));
    NotAGame game;
    return game.run();
}
