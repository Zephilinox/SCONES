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

    // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
    {
        static float f = 0.0f;
        static int counter = 0;

        ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

        ImGui::Text("This is some useful text.");          // Display some text (you can use a format strings too)
        ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bools storing our window open/close state
        ImGui::Checkbox("Another Window", &show_another_window);

        ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

        if (ImGui::Button("Button")) // Buttons return true when clicked (most widgets return true when edited/activated)
            counter++;
        ImGui::SameLine();
        ImGui::Text("counter = %d", counter);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

    // 3. Show another simple window.
    if (show_another_window)
    {
        texture->update([](std::vector<Pixel>& pixels) { pixels = make_texture(); });

        ImGui::Begin("Another Window", &show_another_window); // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
        ImGui::Text("Hello from another window!");
        if (ImGui::Button("Close Me"))
            show_another_window = false;

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
