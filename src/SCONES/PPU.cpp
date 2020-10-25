#include "Bus.hpp"
#include "Framebuffer.hpp"
#include "PPU.hpp"

PPU::PPU(Bus* bus, Framebuffer* fbuffer)
    : addBus(bus)
    , vram(std::make_unique<std::uint8_t[]>(PPU_MAX_ADDRESSABLE_MEMORY))
    , fb(fbuffer)
{
    create_palette();
}

PPU::~PPU()
{
}

void PPU::step()
{
    // Tick the PPU.
    clock++;

    if (clock > 0)
    {
        if (scanline >= 241)
        {
            if (scanline == 241 && clock == 1)
            {
                set_vblank(0x1);
            }
        }
        else
        {
            sprite_evaluation();
            sprite_rendering();
        }
    }

    if (clock >= PPU_SCANLINE_CYCLE_COUNT)
    {
        clock = 0;
        scanline++;
        if (scanline >= PPU_TOTAL_SCANLINES_PER_FRAME - 1)
        {
            scanline = -1;
            set_vblank(0x0);
            frameready = true;
        }
    }
}

void PPU::create_palette()
{
    // Define the standard NES colour palette for rendering.
    palPalette[0x00] = RGB{ 84, 84, 84 };
    palPalette[0x01] = RGB{ 0, 30, 116 };
    palPalette[0x02] = RGB{ 8, 16, 144 };
    palPalette[0x03] = RGB{ 48, 0, 136 };
    palPalette[0x04] = RGB{ 68, 0, 100 };
    palPalette[0x05] = RGB{ 92, 0, 48 };
    palPalette[0x06] = RGB{ 84, 4, 0 };
    palPalette[0x07] = RGB{ 60, 24, 0 };
    palPalette[0x08] = RGB{ 32, 42, 0 };
    palPalette[0x09] = RGB{ 8, 58, 0 };
    palPalette[0x0A] = RGB{ 0, 64, 0 };
    palPalette[0x0B] = RGB{ 0, 60, 0 };
    palPalette[0x0C] = RGB{ 0, 50, 60 };
    palPalette[0x0D] = RGB{ 0, 0, 0 };
    palPalette[0x0E] = RGB{ 0, 0, 0 };
    palPalette[0x0F] = RGB{ 0, 0, 0 };

    palPalette[0x10] = RGB{ 152, 150, 152 };
    palPalette[0x11] = RGB{ 8, 76, 196 };
    palPalette[0x12] = RGB{ 48, 50, 236 };
    palPalette[0x13] = RGB{ 92, 30, 228 };
    palPalette[0x14] = RGB{ 136, 20, 176 };
    palPalette[0x15] = RGB{ 160, 20, 100 };
    palPalette[0x16] = RGB{ 152, 34, 32 };
    palPalette[0x17] = RGB{ 120, 60, 0 };
    palPalette[0x18] = RGB{ 84, 90, 0 };
    palPalette[0x19] = RGB{ 40, 114, 0 };
    palPalette[0x1A] = RGB{ 8, 124, 0 };
    palPalette[0x1B] = RGB{ 0, 118, 40 };
    palPalette[0x1C] = RGB{ 0, 102, 120 };
    palPalette[0x1D] = RGB{ 0, 0, 0 };
    palPalette[0x1E] = RGB{ 0, 0, 0 };
    palPalette[0x1F] = RGB{ 0, 0, 0 };

    palPalette[0x20] = RGB{ 236, 238, 236 };
    palPalette[0x21] = RGB{ 76, 154, 236 };
    palPalette[0x22] = RGB{ 120, 124, 236 };
    palPalette[0x23] = RGB{ 176, 98, 236 };
    palPalette[0x24] = RGB{ 228, 84, 236 };
    palPalette[0x25] = RGB{ 236, 88, 180 };
    palPalette[0x26] = RGB{ 236, 106, 100 };
    palPalette[0x27] = RGB{ 212, 136, 32 };
    palPalette[0x28] = RGB{ 160, 170, 0 };
    palPalette[0x29] = RGB{ 116, 196, 0 };
    palPalette[0x2A] = RGB{ 76, 208, 32 };
    palPalette[0x2B] = RGB{ 56, 204, 108 };
    palPalette[0x2C] = RGB{ 56, 180, 204 };
    palPalette[0x2D] = RGB{ 60, 60, 60 };
    palPalette[0x2E] = RGB{ 0, 0, 0 };
    palPalette[0x2F] = RGB{ 0, 0, 0 };

    palPalette[0x30] = RGB{ 236, 238, 236 };
    palPalette[0x31] = RGB{ 168, 204, 236 };
    palPalette[0x32] = RGB{ 188, 188, 236 };
    palPalette[0x33] = RGB{ 212, 178, 236 };
    palPalette[0x34] = RGB{ 236, 174, 236 };
    palPalette[0x35] = RGB{ 236, 174, 212 };
    palPalette[0x36] = RGB{ 236, 180, 176 };
    palPalette[0x37] = RGB{ 228, 196, 144 };
    palPalette[0x38] = RGB{ 204, 210, 120 };
    palPalette[0x39] = RGB{ 180, 222, 120 };
    palPalette[0x3A] = RGB{ 168, 226, 144 };
    palPalette[0x3B] = RGB{ 152, 226, 180 };
    palPalette[0x3C] = RGB{ 160, 214, 228 };
    palPalette[0x3D] = RGB{ 160, 162, 160 };
    palPalette[0x3E] = RGB{ 0, 0, 0 };
    palPalette[0x3F] = RGB{ 0, 0, 0 };
}

void PPU::set_vblank(std::uint8_t value)
{
    // Set VBlank interval (Signals were not rendering to CPU).
    // PPU does not access memory at this point.
    if (addBus)
    {
        PPUStatusReg status;
        status.data = addBus->read(PPU_ADDRESS_STATUS_REG);
        status.bits.vblank = (1u & value);
        addBus->write(PPU_ADDRESS_STATUS_REG, status.data);
    }
}

void PPU::sprite_evaluation()
{
    if (scanline != 261)
    {
        // sprite evaluation does not occur here.

    }
}

void PPU::sprite_rendering()
{
    // Sprite rendering occurs for each scanline.
    // Fetch each pixel from the shift register every 8 cycles.
    PPURenderControlReg render_control;
    if (addBus)
    {
        render_control.data = addBus->read(PPU_ADDRESS_STATUS_REG);
    }

    // Fetch the pixel from the current tile and draw the
    // sprite.
    if (scanline != -1 && (render_control.bits.background_enable | render_control.bits.sprite_enable))
    {
        // Fetch the bits.
        PPUScrollReg scrollReg;
        scrollReg.data = addBus->read(PPU_ADDRESS_VRAM_REG_1);

        // Priority multiplexer needed here to determine which pixel is extracted.
        (*fb)(clock, scanline, palPalette[0x00]);
    }
}
