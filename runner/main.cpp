//STD
#include <iostream>
#include <thread>

//LIBS
#include <SDL.h>
#include <SCONES/NES.hpp>

int main(int argc, char* args[])
{
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }

    SDL_Window* win = SDL_CreateWindow("Hello World!", 100, 100, 620, 387, SDL_WINDOW_SHOWN);
    if (win == nullptr)
    {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }

    SDL_Renderer* ren
        = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (ren == nullptr)
    {
        std::cerr << "SDL_CreateRenderer Error" << SDL_GetError() << std::endl;
        if (win != nullptr)
        {
            SDL_DestroyWindow(win);
        }
        SDL_Quit();
        return EXIT_FAILURE;
    }

    SDL_Surface* bmp = SDL_LoadBMP("scone.bmp");
    if (bmp == nullptr)
    {
        std::cerr << "SDL_LoadBMP Error: " << SDL_GetError() << std::endl;
        if (ren != nullptr)
        {
            SDL_DestroyRenderer(ren);
        }
        if (win != nullptr)
        {
            SDL_DestroyWindow(win);
        }
        SDL_Quit();
        return EXIT_FAILURE;
    }

    SDL_Texture* tex = SDL_CreateTextureFromSurface(ren, bmp);
    if (tex == nullptr)
    {
        std::cerr << "SDL_CreateTextureFromSurface Error: " << SDL_GetError() << std::endl;
        if (bmp != nullptr)
        {
            SDL_FreeSurface(bmp);
        }
        if (ren != nullptr)
        {
            SDL_DestroyRenderer(ren);
        }
        if (win != nullptr)
        {
            SDL_DestroyWindow(win);
        }
        SDL_Quit();
        return EXIT_FAILURE;
    }
    SDL_FreeSurface(bmp);

    for (int i = 0; i < 20; i++)
    {
        SDL_RenderClear(ren);
        SDL_RenderCopy(ren, tex, nullptr, nullptr);
        SDL_RenderPresent(ren);
        SDL_Delay(100);
    }

    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();

    NES nes;

    while (true)
    {
        nes.tick();
        std::cout << "clock: " << nes.get_clock() << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds{ 16 });
    }

    return 0;
}