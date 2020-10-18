#pragma once

//STD
#include <cstdint>
#include <array>

//SELF
#include "Bus.hpp"

//todo: maybe I don't need this :shrug:
enum InstructionType
{
    None = 0,
    ADC, //Add Memory to Accumulator with Carry
    AND, //AND Memory with Accumulator
    ASIL, //Shift Left One Bit (Memory or Accumulator)
    BCC, //Branch on Carry Clear
    BCS, //Branch on Carry Set
    BEQ, //Branch on Result Zero
    BIT, //Test Bits in Memory with Accumulator
    BMI, //Branch on Result Minus
    BNE, //Branch on Result not Zero
    BPL, //Branch on Result Plus
    BRK, //Force Break
    BVC, //Branch on Overflow Clear
    BVS, //Branch on Overflow Set
    CLC, //Clear Carry Flag
    CLD, //Clear Decimal Mode
    CLI, //CLear Interrupt Disable Bit
    CLV, //Clear Overflow Flag
    CMP, //Compare Memory and Accumulator
    CPX, //Compare Memory and Index X
    CPY, //Comapre Memory and Index Y
    DEC, //Decrement Memory by One
    DEX, //Decrement Index X by One
    DEY, //Decrement Index Y by One
    EOR, //XOR Memory with Accumulator
    INC, //Increment Memory by One
    INX, //Increment Index X by One
    INY, //Increment Index Y by One
    JMP, //Jump to New Location
    JSR, //Jump to New Location Saving Return Address
    LDA, //Load Accumulator with Memory
    LDX, //Load Index X with Memory
    LDY, //Load Index Y with Memory
    LSR, //Shift One Bit Right (Memory or Accumulator)
    NOP, //No Operation
    ORA, //OR Memory with Accumulator
    PHA, //Push Accumulator on Stack
    PHP, //Push Processor Status on Stack
    PLA, //Pull Accumulator from Stack
    PLP, //Pull Processor Status from Stack
    ROL, //Rotate One Bit Left (Memory or Accumulator)
    ROR, //Rotate One Bit Right (Memory or Accumulator)
    RTI, //Return from Interrupt
    RTS, //Return from Subroutine
    SBC, //Subtract Memory from Accumulator with Borrow
    SEC, //Set Carry Flag
    SED, //Set Decimal Mode
    SEI, //Set Interrupt Disable Status
    STA, //Store Accumulator in Memory
    STX, //Store Index X in Memory
    STY, //Store Index Y in Memory
    TAX, //Transfer Accumulator to Index X
    TAY, //Transfer Accumulator to Index Y
    TSX, //Transfer Stack Pointer to Index X
    TXA, //Transfer Index X to Accumulator
    TXS, //Transfer Index X to Stack Register
    TYA, //Transfer Index Y to Accumulator
};

class CPU;

class Instruction
{
public:
    using InstructionFunction = std::uint8_t(CPU::*)();

    InstructionFunction execute;
    std::uint8_t bytes = 0;
    std::uint8_t base_cycles = 0;

private:

};

/*
 *  ready byte at program counter location
 *  use byte to index instruction(opcode) table, giving addressing mode and cycles
 *  read up to 2 more bytes
 *  execute the instruction
 *  wait, count cycles, complete
 */

enum class StatusRegisterFlags : std::uint8_t
{
    // clang-format off
    None                = 0,
    Carry               = 1 << 0,
    Zero                = 1 << 1,
    DisableInterrupts   = 1 << 2,
    DecimalMode         = 1 << 3,
    Break               = 1 << 4,
    Unused              = 1 << 5,
    Overflow            = 1 << 6,
    Negative            = 1 << 7,
    // clang-format on
};

class CPU
{
public:
    using AddressModeFunction = std::uint8_t (CPU::*)();
    using InstructionTable = std::array<Instruction, 256>;

    CPU(Bus* bus);

    void reset();
    void interrupt_request();
    void force_interrupt_request();
    void step();
    bool last_instruction_complete();

private:
    bool get_flag(StatusRegisterFlags flag);
    void set_flag(StatusRegisterFlags flag, bool value);

    std::uint8_t read_from_memory(std::uint16_t address);
    void write_to_memory(std::uint16_t address, std::uint8_t data);

    //The read location could be a memory address or part of the instruction
    std::uint8_t fetch();
    
    constexpr InstructionTable generate_instruction_table() const;

    // clang-format off
    template<AddressModeFunction AddressMode> std::uint8_t instruction_adc();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_and();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_asl();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_bcc();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_bcs();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_beq();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_bit();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_bmi();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_bne();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_bpl();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_brk();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_bvc();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_bvs();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_clc();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_cld();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_cli();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_clv();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_cmp();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_cpx();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_cpy();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_dec();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_dex();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_dey();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_eor();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_inc();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_inx();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_iny();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_jmp();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_jsr();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_lda();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_ldx();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_ldy();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_lsr();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_nop();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_ora();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_pha();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_php();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_pla();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_plp();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_rol();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_ror();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_rti();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_rts();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_sbc();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_sec();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_sed();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_sei();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_sta();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_stx();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_sty();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_tax();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_tay();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_tsx();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_txa();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_txs();
    template<AddressModeFunction AddressMode> std::uint8_t instruction_tya();

    template <CPU::AddressModeFunction AddressMode, StatusRegisterFlags flag, bool set> std::uint8_t instruction_branch();
    template <CPU::AddressModeFunction AddressMode> std::uint8_t instruction_modify_register(std::uint8_t& target_register, std::int8_t value);
    template <CPU::AddressModeFunction AddressMode> std::uint8_t instruction_transfer(std::uint8_t& source, std::uint8_t& target);
    template <CPU::AddressModeFunction AddressMode> std::uint8_t instruction_load_register(std::uint8_t& target_register);
    template <CPU::AddressModeFunction AddressMode> std::uint8_t instruction_compare(std::uint8_t& target_register);
    // clang-format on

    std::uint8_t address_mode_implied();
    std::uint8_t address_mode_immidiate();
    std::uint8_t address_mode_accumulator();
    std::uint8_t address_mode_relative();
    std::uint8_t address_mode_zero_page();
    std::uint8_t address_mode_zero_page_y();
    std::uint8_t address_mode_zero_page_x();
    std::uint8_t address_mode_absolute();
    std::uint8_t address_mode_absolute_x();
    std::uint8_t address_mode_absolute_y();
    std::uint8_t address_mode_indirect();
    std::uint8_t address_mode_indirect_x();
    std::uint8_t address_mode_indirect_y();

    std::uint8_t unofficial_opcode();

    Bus* bus = nullptr;

    std::uint8_t register_accumulator = 0;
    std::uint8_t register_x = 0;
    std::uint8_t register_y = 0;
    std::uint8_t register_status{};
    //points to a memory address that is incremented/decremented when pushing/pulling from the stack
    std::uint8_t stack_pointer = 0;
    //Stores the address of the next program byte
    std::uint16_t program_counter = 0;

    std::uint8_t fetched = 0; //Represents the working input value to the ALU
    std::uint16_t temp = 0; //A convenience variable used everywhere
    std::uint16_t address_absolute; //All used memory addresses end up in here
    std::uint16_t address_relative; //Represents the absolute address following a branch
    std::uint8_t opcode; //Instruction byte
    std::uint8_t remaining_cycles; //For the last executed instruction
    std::uint32_t clock_count = 0; //Number of clock cycles executed

    InstructionTable instruction_table;
};

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_adc()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();

    std::uint16_t add = static_cast<std::uint16_t>(register_accumulator) + static_cast<std::uint16_t>(fetched) + static_cast<std::uint16_t>(get_flag(StatusRegisterFlags::Carry));
    const bool over_a_byte = add > 255;
    const bool byte_is_zero = (add & 0x00FF) == 0;
    const bool negative = add & 0x80;
    //yikes, serious magic here that I don't understand
    const bool overflow = (~(static_cast<std::uint16_t>(register_accumulator) ^ static_cast<std::uint16_t>(fetched) & (static_cast<std::uint16_t>(register_accumulator) ^ add)) & 0x0080);
    set_flag(StatusRegisterFlags::Carry, over_a_byte);
    set_flag(StatusRegisterFlags::Zero, byte_is_zero);
    set_flag(StatusRegisterFlags::Negative, negative);
    set_flag(StatusRegisterFlags::Overflow, overflow);

    register_accumulator = add & 0x00FF;

    return 1 + extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_and()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();
    register_accumulator = register_accumulator & fetched;
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);
    return 1 + extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_asl()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();
    auto result = static_cast<std::uint16_t>(fetched) << 1;
    const bool carry = (result & 0xFF00) > 0;
    set_flag(StatusRegisterFlags::Carry, carry);
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);

    if constexpr (AddressMode == &CPU::address_mode_implied)
        register_accumulator = result & 0x00FF;
    else
        write_to_memory(address_absolute, result & 0x00FF);

    return extra_cycles_from_addressing;
}

template <CPU::AddressModeFunction AddressMode, StatusRegisterFlags flag, bool set>
std::uint8_t CPU::instruction_branch()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    if (get_flag(flag) == set)
        return extra_cycles_from_addressing;

    clock_count++;
    address_absolute = program_counter + address_relative;
    std::uint16_t address_absolute_page = address_absolute & 0xFF00;
    std::uint16_t program_counter_page = program_counter & 0xFF00;

    //different page costs a cycle
    if (address_absolute_page != program_counter_page)
        clock_count++;

    program_counter = address_absolute;
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_bcc()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Carry, true>();
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_bcs()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Carry, false>();
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_beq()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Zero, true>();
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_bit()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();

    auto result = register_accumulator & fetched;

    const bool byte_is_zero = (result & 0x00FF) == 0;
    const bool overflow = fetched & (1 << 6);
    const bool negative = fetched & (1 << 7);
    set_flag(StatusRegisterFlags::Zero, byte_is_zero);
    set_flag(StatusRegisterFlags::Overflow, overflow);
    set_flag(StatusRegisterFlags::Negative, negative);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_bmi()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Negative, true>();
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_bne()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Zero, false>();
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_bpl()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Negative, false>();
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_brk()
{
    constexpr auto stack_address = 0x0100;

    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    program_counter++;

    set_flag(StatusRegisterFlags::DisableInterrupts, true);

    auto program_counter_low_byte = program_counter & 0x00FF;
    auto program_counter_high_byte = (program_counter >> 8) & 0x00FF;
    write_to_memory(stack_address + stack_pointer, program_counter_high_byte);
    stack_pointer++;
    write_to_memory(stack_address + stack_pointer, program_counter_low_byte);
    stack_pointer++;

    set_flag(StatusRegisterFlags::Break, true);
    write_to_memory(stack_address + stack_pointer, register_status);
    stack_pointer--;
    set_flag(StatusRegisterFlags::Break, false);

    //todo: magic address
    std::uint16_t low_byte = read_from_memory(0xFFFE);
    std::uint16_t high_byte = read_from_memory(0xFFFF);
    high_byte = high_byte << 8;
    program_counter = high_byte | low_byte;

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_bvc()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Overflow, false>();
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_bvs()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Overflow, true>();
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_clc()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    set_flag(StatusRegisterFlags::Carry, false);
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_cld()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    set_flag(StatusRegisterFlags::DecimalMode, false);
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_cli()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    set_flag(StatusRegisterFlags::DisableInterrupts, false);
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_clv()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    set_flag(StatusRegisterFlags::Overflow, false);
    return extra_cycles_from_addressing;
}

template <CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_compare(std::uint8_t& target_register)
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();

    auto result = static_cast<std::uint16_t>(target_register) - static_cast<std::uint16_t>(fetched);
    set_flag(StatusRegisterFlags::Carry, target_register >= fetched);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0x0000);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_cmp()
{
    return instruction_compare<AddressMode>(register_accumulator);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_cpx()
{
    return instruction_compare<AddressMode>(register_x);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_cpy()
{
    return instruction_compare<AddressMode>(register_y);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_dec()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();

    auto result = fetched - 1;
    write_to_memory(address_absolute, result & 0x00FF);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);

    return extra_cycles_from_addressing;
}

template <CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_modify_register(std::uint8_t& target_register, std::int8_t value)
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    target_register += value;
    set_flag(StatusRegisterFlags::Zero, target_register == 0);
    set_flag(StatusRegisterFlags::Negative, target_register & 0x80);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_dex()
{
    return instruction_modify_register<AddressMode>(register_x, -1);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_dey()
{
    return instruction_modify_register<AddressMode>(register_y, -1);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_eor()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();

    register_accumulator = register_accumulator ^ fetched;
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);

    return 1 + extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_inc()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();

    auto result = fetched + 1;
    write_to_memory(address_absolute, result & 0x00FF);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_inx()
{
    return instruction_modify_register<AddressMode>(register_x, 1);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_iny()
{
    return instruction_modify_register<AddressMode>(register_y, 1);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_jmp()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    program_counter = address_absolute;
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_jsr()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    program_counter--;

    constexpr auto stack_address = 0x0100;
    auto program_counter_low_byte = program_counter & 0x00FF;
    auto program_counter_high_byte = (program_counter >> 8) & 0x00FF;
    write_to_memory(stack_address + stack_pointer, program_counter_high_byte);
    stack_pointer--;
    write_to_memory(stack_address + stack_pointer, program_counter_low_byte);
    stack_pointer--;

    program_counter = address_absolute;

    return extra_cycles_from_addressing;
}

template <CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_load_register(std::uint8_t& target_register)
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();
    target_register = fetched;
    set_flag(StatusRegisterFlags::Zero, target_register == 0);
    set_flag(StatusRegisterFlags::Negative, target_register & 0x80);

    return 1 + extra_cycles_from_addressing;
}

//todo: do this within the instruction table instead? or is that too noisy
template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_lda()
{
    return instruction_load_register<AddressMode>(register_accumulator);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_ldx()
{
    return instruction_load_register<AddressMode>(register_x);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_ldy()
{
    return instruction_load_register<AddressMode>(register_y);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_lsr()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();
    set_flag(StatusRegisterFlags::Carry, fetched & 0x0001);

    std::uint16_t result = fetched >> 1;

    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);

    if constexpr (AddressMode == &CPU::address_mode_implied)
        register_accumulator = result & 0x00FF;
    else
        write_to_memory(address_absolute, result & 0x00FF);

    return extra_cycles_from_addressing;
}

//todo: make better
template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_nop()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    switch (opcode)
    {
        case 0x1C:
        case 0x3C:
        case 0x5C:
        case 0x7C:
        case 0xDC:
        case 0xFC:
            return 1 + extra_cycles_from_addressing;
    }

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_ora()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();
    register_accumulator = register_accumulator & fetched;
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);

    return 1 + extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_pha()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    constexpr auto stack_address = 0x0100;
    write_to_memory(stack_address + stack_pointer, register_accumulator);
    stack_pointer--;

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_php()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    constexpr auto stack_address = 0x0100;
    write_to_memory(stack_address + stack_pointer, register_status | static_cast<std::uint8_t>(StatusRegisterFlags::Break) | static_cast<std::uint8_t>(StatusRegisterFlags::Unused));
    set_flag(StatusRegisterFlags::Break, false);
    set_flag(StatusRegisterFlags::Unused, false);
    stack_pointer--;

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_pla()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    constexpr auto stack_address = 0x0100;
    stack_pointer++;
    register_accumulator = read_from_memory(stack_address + stack_pointer);
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_plp()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    constexpr auto stack_address = 0x0100;
    stack_pointer++;
    register_status = read_from_memory(stack_address + stack_pointer);
    set_flag(StatusRegisterFlags::Unused, true);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_rol()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();

    std::uint16_t result = static_cast<std::uint16_t>(fetched << 1) | static_cast<std::uint16_t>(get_flag(StatusRegisterFlags::Carry));
    set_flag(StatusRegisterFlags::Carry, result & 0xFF00);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, (result & 0x0080) == 0);
    if constexpr (AddressMode == &CPU::address_mode_implied)
        register_accumulator = result & 0x00FF;
    else
        write_to_memory(address_absolute, result & 0x00FF);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_ror()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();

    std::uint16_t result = static_cast<std::uint16_t>(static_cast<std::uint16_t>(get_flag(StatusRegisterFlags::Carry)) << 7) | static_cast<std::uint16_t>(fetched >> 1);
    set_flag(StatusRegisterFlags::Carry, fetched & 0x01);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);
    if constexpr (AddressMode == &CPU::address_mode_implied)
        register_accumulator = result & 0x00FF;
    else
        write_to_memory(address_absolute, result & 0x00FF);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_rti()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    constexpr auto stack_address = 0x0100;

    stack_pointer++;
    register_status = read_from_memory(stack_address + stack_pointer);
    register_status = register_status & ~static_cast<std::uint8_t>(StatusRegisterFlags::Break);
    register_status = register_status & ~static_cast<std::uint8_t>(StatusRegisterFlags::Unused);

    stack_pointer++;
    program_counter = static_cast<std::uint16_t>(read_from_memory(stack_address + stack_pointer));
    stack_pointer++;
    program_counter = program_counter | (static_cast<std::uint16_t>(read_from_memory(stack_address + stack_pointer)) << 8);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_rts()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    constexpr auto stack_address = 0x0100;

    stack_pointer++;
    program_counter = static_cast<std::uint16_t>(read_from_memory(stack_address + stack_pointer));
    stack_pointer++;
    program_counter = program_counter | (static_cast<std::uint16_t>(read_from_memory(stack_address + stack_pointer)) << 8);

    program_counter++;
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_sbc()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    fetch();
    std::uint16_t result = static_cast<std::uint16_t>(fetched) & 0x00FF;

    //same as ADC
    std::uint16_t add = static_cast<std::uint16_t>(register_accumulator) + static_cast<std::uint16_t>(result) + static_cast<std::uint16_t>(get_flag(StatusRegisterFlags::Carry));
    const bool over_a_byte = add > 255;
    const bool byte_is_zero = (add & 0x00FF) == 0;
    const bool negative = add & 0x80;
    //yikes, serious magic here that I don't understand
    const bool overflow = (add ^ static_cast<std::uint16_t>(register_accumulator) & (add ^ result) & 0x0080);
    set_flag(StatusRegisterFlags::Carry, over_a_byte);
    set_flag(StatusRegisterFlags::Zero, byte_is_zero);
    set_flag(StatusRegisterFlags::Negative, negative);
    set_flag(StatusRegisterFlags::Overflow, overflow);

    register_accumulator = add & 0x00FF;
    return 1 + extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_sec()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    set_flag(StatusRegisterFlags::Carry, true);
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_sed()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    set_flag(StatusRegisterFlags::DecimalMode, true);
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_sei()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    set_flag(StatusRegisterFlags::DisableInterrupts, true);
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_sta()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    write_to_memory(address_absolute, register_accumulator);
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_stx()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    write_to_memory(address_absolute, register_x);
    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_sty()
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();
    write_to_memory(address_absolute, register_y);
    return extra_cycles_from_addressing;
}

template <CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_transfer(std::uint8_t& source, std::uint8_t& target)
{
    std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

    target = source;
    set_flag(StatusRegisterFlags::Zero, target == 0);
    set_flag(StatusRegisterFlags::Negative, target & 0x80);

    return extra_cycles_from_addressing;
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_tax()
{
    return instruction_transfer<AddressMode>(register_accumulator, register_x);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_tay()
{
    return instruction_transfer<AddressMode>(register_accumulator, register_y);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_tsx()
{
    return instruction_transfer<AddressMode>(stack_pointer, register_x);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_txa()
{
    return instruction_transfer<AddressMode>(register_x, register_accumulator);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_txs()
{
    return instruction_transfer<AddressMode>(register_x, stack_pointer);
}

template<CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::instruction_tya()
{
    return instruction_transfer<AddressMode>(register_y, register_accumulator);
}