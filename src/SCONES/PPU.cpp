#include "Bus.hpp"
#include "Framebuffer.hpp"
#include "PPU.hpp"

PPU::PPU(Bus* bus, Framebuffer* fbuffer)
    : addBus(bus)
    , oam(std::make_unique<std::uint8_t[]>(PPU_MAX_OAM_REG_SIZE))
    , vram(std::make_unique<std::uint8_t[]>(PPU_VRAM_SIZE))
    , pallete_ram(std::make_unique<std::uint8_t[]>(PPU_PALLETE_COUNT))
    , fb(fbuffer)
{
    create_palette();
    addBus->connect_ppu(this);
}

PPU::~PPU()
{
}

void PPU::step()
{
    // Tick the PPU.
    clock++;

    if (scanline == 241)
    {
        PPUSTATUS.bits.vblank = 1;
        nmi = (PPUCTRL.bits.nmi_enable ? 1 : 0);
    }

    if (clock > 0)
    {
        if (clock == 1 && scanline == -1)
        {
            PPUSTATUS.bits.vblank = 0;
        }
    }

    if (clock >= PPU_SCANLINE_CYCLE_COUNT)
    {
        clock = 0;
        scanline++;
        if (scanline >= PPU_TOTAL_SCANLINES_PER_FRAME - 1)
        {
            scanline = -1;
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

void PPU::ppu_data_reg_write(std::uint8_t data)
{
    // Writes data to current VRAM address.
    ppu_write(vram_rag.data, data);
    vram_rag.data += (PPUCTRL.bits.increment_mode ? 32 : 1);
}

void PPU::ppu_scroll_reg_write(std::uint8_t data)
{
    if (address_latch == 0x0)
    {
        fine_x_scroll = (data & 0x7);
        tvram_reg.bits.course_x_scroll = (data >> 3);
    }
    else
    {
        tvram_reg.bits.fine_y_scroll = (data & 0x7);
        tvram_reg.bits.course_y_scroll = (data >> 3);
    }
    address_latch = !address_latch;
}

void PPU::ppu_address_reg_write(std::uint8_t data)
{
    // Here we write to either the low or high portion of the address with our data.
    tvram_reg.data = (tvram_reg.data & (address_latch == 0x0 ? 0x00FF : 0xFF00)) | (data << (address_latch & 0x8));

    // Once full register has been written we swap the tram register to the VRAM register.
    // Simulates the two write operations for the register that occur due to the 8bit bus.
    if (address_latch)
    {
        vram_rag = tvram_reg;
    }
    address_latch = !address_latch;
}

void PPU::ppu_address_status_reg_read(std::uint8_t& data)
{
    data = PPUSTATUS.data;
    PPUSTATUS.bits.vblank = 0;
    address_latch = 0x0;
}

void PPU::ppu_data_reg_read(std::uint8_t& data)
{
    // Reads data from current VRAM address.
    data = bus_read(vram_rag.data);
    vram_rag.data += (PPUCTRL.bits.increment_mode ? 32 : 1);
}

std::uint8_t PPU::ppu_read(std::uint16_t address)
{
    std::uint8_t data = 0x0;
    if (address <= 0x1FFF)
    {
        if (bus_cart)
        {
            // TODO - Fetch data from the cartridge based on mapper.
        }
    }
    else if (address >= 0x2000 && address <= 0x3EFF)
    {
        // TODO - Test this memory access.
        std::uint16_t base_add = (address >= 0x3000) ? 0x3000 : 0x2000;
        auto add_rel = address - base_add;
        data = vram[add_rel];
    }
    else if (address >= 0x3F00)
    {
        // Reads palette indexes.
        // Therefore we will return the index from our colour palette.
        // The calling routiene can then use this number to get the RGB value.
        auto adrr = ((address - 0x3F00) % 0x0020);
        pallete_ram[adrr] = data;
    }

    return data;
}

void PPU::ppu_write(std::uint16_t address, std::uint8_t data)
{
    if (address <= 0x1FFF)
    {
        if (bus_cart)
        {
            // TODO - Fetch data from the cartridge based on mapper.
        }
    }
    else if (address >= 0x2000 && address <= 0x3EFF)
    {
        std::uint16_t base_add = (address >= 0x3000) ? 0x3000 : 0x2000;
        auto add_rel = address - base_add;
        vram[add_rel] = data;
    }
    else
    {
        // Reads palette indexes.
        // Therefore we will return the index from our colour palette.
        // The calling routiene can then use this number to get the RGB value.
        auto adrr = ((address - 0x3F00) % 0x0020);
        pallete_ram[adrr] = data;
    }
}

void PPU::bus_write(std::uint16_t address, std::uint8_t data)
{
    // TODO - Rework registers here.
    switch (address)
    {
    case PPU_ADDRESS_PPUCTRL_REG:
        PPUCTRL.data = data;
        break;
    case PPU_ADDRESS_MASK_REG:
        PPUMASK.data = data;
    case PPU_ADDRESS_STATUS_REG: // Not writable.
        break;
    case PPU_ADDRESS_SCROLL_REG:
        ppu_scroll_reg_write(data);
        break;
    case PPU_ADDRESS_ADDR_REG:
        ppu_address_reg_write(data);
        break;
    case PPU_ADDRESS_DATA_REG:
        bus_write(address, data);
        break;
    default: // Dont do anything.
        break;
    }
}

std::uint8_t PPU::bus_read(std::uint16_t address)
{
    std::uint8_t data = 0;
    switch (address)
    {
    case PPU_ADDRESS_PPUCTRL_REG: // Not readable.
        break;
    case PPU_ADDRESS_MASK_REG: // Not readable.
        break;
    case PPU_ADDRESS_STATUS_REG:
        ppu_address_status_reg_read(data);
        break;
    case PPU_ADDRESS_ADDR_REG: // Not readable.
        break;
    case PPU_ADDRESS_DATA_REG:
        ppu_data_reg_read(data);
        break;
    default:
        data = 0;
        break;
    }

    return data;
}

void PPU::get_pallete_contents(Framebuffer* fb)
{
    // TODO - Write the contents of the pallete memory here. 
}
