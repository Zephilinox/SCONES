#include "NotAGame.hpp"

//STD

//LIBS

//SELF

using namespace paperbag;

constexpr int size = 512;

std::vector<Pixel> make_texture()
{
    std::vector<Pixel> pixels;
    pixels.resize(size * size);
    for (int i = 0; i < size * size; ++i)
    {
        //rand() is slow, performance will be better for real :)
        std::uint8_t r = std::rand() % 255;
        std::uint8_t g = std::rand() % 255;
        std::uint8_t b = std::rand() % 255;
        pixels[i] = Pixel{ r, g, b, 255 };
    }

    return pixels;
}

NotAGame::NotAGame()
    : window({"SCONES - Emulator"})
    , renderer(&window)
    , gui(&window, &renderer)
    , texture(renderer.make_texture(make_texture(), size, size))
{
    cartridge_path.resize(1024);
    cartridge_path = "../../tests/resources/nestest.nes";
}

int NotAGame::run()
{
    while (!done)
    {
        input();
        update();
        render();
    }

    return 0;
}

void NotAGame::input()
{
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
    // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        gui.input(event);

        if (event.type == SDL_QUIT)
            done = true;

        if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window.get_sdl_handle()))
            done = true;
    }
}

void NotAGame::update()
{
    gui.update();

    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    {
        ImGui::Begin("Settings");

        ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bools storing our window open/close state
        ImGui::Checkbox("NES Screen", &show_nes_screen);
        ImGui::Checkbox("NES Screen - Pause when not active", &pause_nes_when_not_active);
        ImGui::Checkbox("NES Screen - Update every cycle", &update_nes_screen_every_cycle);

        if (ImGui::Button("NES Step"))
            nes.tick();

        ImGui::SameLine();
        ImGui::Text("NES clock = %d", nes.get_clock());

        std::string buffer = cartridge_path;
        buffer.resize(1024);
        ImGui::InputText("NES Cartridge", buffer.data(), buffer.size());
        cartridge_path = buffer;

        if (ImGui::Button("NES Insert Cartridge"))
            nes.insert_cartridge(cartridge_path);

        if (ImGui::Button("NES Reset"))
            nes.reset();

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

    if (show_nes_screen)
    {
        ImGui::Begin("NES", &show_nes_screen);
        if (!pause_nes_when_not_active || ImGui::IsWindowFocused())
        {
            if (!update_nes_screen_every_cycle)
            {
                while (!nes.frame_ready())
                    nes.tick();
            }
            else
            {
                nes.tick();
            }

            texture->update([](std::vector<Pixel>& pixels) {
                pixels = make_texture();
            });
        }

        ImGui::Image((void*)(intptr_t)texture->get_opengl_texture_id(), ImVec2(texture->get_width(), texture->get_height()));
        ImGui::End();
    }
}

void NotAGame::render()
{
    window.clear(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    gui.render();
    renderer.end();
    window.end();
}
