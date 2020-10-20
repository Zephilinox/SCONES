#pragma once

#include <cstdint>
#include <memory>

constexpr std::int32_t ADDRESS_BUS_RAM_SIZE = 64 * 1024; 


class Bus
{
public:
   Bus();
  ~Bus() = default;

  std::uint8_t read(std::uint16_t address);
  void write(std::uint16_t address, std::uint8_t data);

private: 
	std::unique_ptr<std::uint8_t[]> ram;
};