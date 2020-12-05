#include "NotAGame.hpp"

//STD

//LIBS

//SELF
#include "Paperbag/Screens/MemoryEditor.hpp"

using namespace paperbag;

constexpr int size = 512;

std::vector<Pixel> make_texture(int w, int h)
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
    , texture(renderer.make_texture(make_texture(size, size), size, size))
    , fb_texture(renderer.make_texture(make_texture(nes.get_debug_framebuffer()->get_width(), nes.get_debug_framebuffer()->get_height()),
        nes.get_debug_framebuffer()->get_width(), nes.get_debug_framebuffer()->get_height()))
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

    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    {
        ImGui::Begin("Settings");

        ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bools storing our window open/close state
        ImGui::Checkbox("NES CPU Memory Editor", &show_nes_cpu_memory_editor);
        ImGui::Checkbox("NES CPU - Update per instruction, not cycle", &update_nes_per_instruction);
        ImGui::InputInt("Nes CPU - Updates per frame", &nes_updates_per_frame, 1, 10);
        if (nes_updates_per_frame < 1)
            nes_updates_per_frame = 1;
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

        {
            auto op = fmt::format("NES CPU OP = {:02X}", nes.get_cpu().get_opcode());
            auto pc = fmt::format("NES CPU PC = {:04X}", nes.get_cpu().get_program_counter());
            auto a = fmt::format("NES CPU A = {:02X}", nes.get_cpu().get_register_accumulator());
            auto x = fmt::format("NES CPU X = {:02X}", nes.get_cpu().get_register_x());
            auto y = fmt::format("NES CPU Y = {:02X}", nes.get_cpu().get_register_y());
            auto p = fmt::format("NES CPU P = {:02X}", nes.get_cpu().get_processor_status());
            auto sp = fmt::format("NES CPU SP = {:02X}", nes.get_cpu().get_stack_pointer());
            auto abs = fmt::format("NES CPU ABS = {:04X}", nes.get_cpu().get_absolute_address());
            auto f = fmt::format("NES CPU F = {:02X}", nes.get_cpu().get_fetched());

            ImGui::Text(op.c_str());
            ImGui::Text(pc.c_str());
            ImGui::Text(a.c_str());
            ImGui::Text(x.c_str());
            ImGui::Text(y.c_str());
            ImGui::Text(p.c_str());
            ImGui::Text(sp.c_str());
            ImGui::Text(abs.c_str());
            ImGui::Text(f.c_str());
        }

        buffer = program_counter_value;
        buffer.resize(1024);
        ImGui::InputText("Program Counter", buffer.data(), buffer.size());
        program_counter_value = buffer;

        if (ImGui::Button("NES CPU Set Program Counter"))
        {
            int pc;
            sscanf(program_counter_value.c_str(), "%x", &pc);
            nes.get_cpu().set_program_counter(pc);
        }

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
                for (int i = 0; i < nes_updates_per_frame; ++i)
                {
                    if (update_nes_per_instruction)
                    {
                        std::uint8_t last_op = nes.get_cpu().get_opcode();
                        while (last_op == nes.get_cpu().get_opcode())
                        {
                            last_op = nes.get_cpu().get_opcode();
                            nes.tick();
                        } 
                    }
                    else
                    {
                        nes.tick();
                    }
                }
            }

            texture->update([](std::vector<Pixel>& pixels) {
                pixels = make_texture(size, size);
            });
            if (update_nes_screen_every_cycle) {
                fb_texture->update([&](std::vector<Pixel>& pixels) {
                    const Framebuffer* fb = nes.get_debug_framebuffer();
                    for (int y = 0; y < fb->get_height(); y++)
                    {
                        for (int x = 0; x < fb->get_width(); x++)
                        {
                            pixels[x + (y * fb->get_width())] = Pixel{
                                (*fb)(x, y).r, (*fb)(x, y).g, (*fb)(x, y).b, 255
                            };
                        }
                    }
                });
            }

        }

        ImGui::Image((void*)(intptr_t)texture->get_opengl_texture_id(), ImVec2(texture->get_width(), texture->get_height()));
        ImGui::Text("PPU Framebuffer:");
        ImGui::Image((void*)(intptr_t)fb_texture->get_opengl_texture_id(), ImVec2(fb_texture->get_width(), fb_texture->get_height()));
        ImGui::End();
    }

    if (show_nes_cpu_memory_editor)
    {
        static MemoryEditor memory_editor;

        ImGui::Begin("CPU Memory Editor", &show_nes_cpu_memory_editor);
        auto* data = nes.get_bus().get_cpu_ram().data();
        auto size = nes.get_bus().get_cpu_ram().size() * sizeof(std::uint8_t);
        memory_editor.DrawContents(data, size);
        ImGui::End();
    }

    // PPU Memory Editor
    {
        static MemoryEditor memory_editor;
        uint32_t memorySize = 0;

        ImGui::Begin("PPU Memory Editor", &show_nes_cpu_memory_editor);
        auto* data = nes.get_ppu().get_vram(memorySize);
        auto size = memorySize * sizeof(std::uint8_t);
        memory_editor.DrawContents(data, size);
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
