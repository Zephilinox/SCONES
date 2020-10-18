#include "BUS.hpp"

Bus::Bus()
    : ram(std::make_unique<std::uint8_t[]>(ADDRESS_BUS_RAM_SIZE))
{
}

std::uint8_t Bus::read(std::uint16_t add)
{
    if (add >= 0x0000 && add <= 0x00FF) {
        return ram.get()[add];
    }

    return 0x0;
}

void Bus::write(std::uint16_t add, std::uint8_t data)
{
    if (add >= 0x0000 && add <= 0x00FF) {
        ram.get()[add] = data;
    }
}