#pragma once

//STD
#include <cstdint>
#include <array>

//LIBS
#include <spdlog/spdlog.h>

//SELF
#include "Bus.hpp"

enum class InstructionType
{
    None = 0,
    ADC,  //Add Memory to Accumulator with Carry
    AND,  //AND Memory with Accumulator
    ASL, //Shift Left One Bit (Memory or Accumulator)
    BCC,  //Branch on Carry Clear
    BCS,  //Branch on Carry Set
    BEQ,  //Branch on Result Zero
    BIT,  //Test Bits in Memory with Accumulator
    BMI,  //Branch on Result Minus
    BNE,  //Branch on Result not Zero
    BPL,  //Branch on Result Plus
    BRK,  //Force Break
    BVC,  //Branch on Overflow Clear
    BVS,  //Branch on Overflow Set
    CLC,  //Clear Carry Flag
    CLD,  //Clear Decimal Mode
    CLI,  //CLear Interrupt Disable Bit
    CLV,  //Clear Overflow Flag
    CMP,  //Compare Memory and Accumulator
    CPX,  //Compare Memory and Index X
    CPY,  //Comapre Memory and Index Y
    DEC,  //Decrement Memory by One
    DEX,  //Decrement Index X by One
    DEY,  //Decrement Index Y by One
    EOR,  //XOR Memory with Accumulator
    INC,  //Increment Memory by One
    INX,  //Increment Index X by One
    INY,  //Increment Index Y by One
    JMP,  //Jump to New Location
    JSR,  //Jump to New Location Saving Return Address
    LDA,  //Load Accumulator with Memory
    LDX,  //Load Index X with Memory
    LDY,  //Load Index Y with Memory
    LSR,  //Shift One Bit Right (Memory or Accumulator)
    NOP,  //No Operation
    ORA,  //OR Memory with Accumulator
    PHA,  //Push Accumulator on Stack
    PHP,  //Push Processor Status on Stack
    PLA,  //Pull Accumulator from Stack
    PLP,  //Pull Processor Status from Stack
    ROL,  //Rotate One Bit Left (Memory or Accumulator)
    ROR,  //Rotate One Bit Right (Memory or Accumulator)
    RTI,  //Return from Interrupt
    RTS,  //Return from Subroutine
    SBC,  //Subtract Memory from Accumulator with Borrow
    SEC,  //Set Carry Flag
    SED,  //Set Decimal Mode
    SEI,  //Set Interrupt Disable Status
    STA,  //Store Accumulator in Memory
    STX,  //Store Index X in Memory
    STY,  //Store Index Y in Memory
    TAX,  //Transfer Accumulator to Index X
    TAY,  //Transfer Accumulator to Index Y
    TSX,  //Transfer Stack Pointer to Index X
    TXA,  //Transfer Index X to Accumulator
    TXS,  //Transfer Index X to Stack Register
    TYA,  //Transfer Index Y to Accumulator
};

std::string instruction_type_to_string(InstructionType type);

class CPU;

class Instruction
{
public:
    using InstructionFunction = bool(CPU::*)();

    InstructionType type;
    InstructionFunction execute;
    std::uint8_t bytes = 0;
    std::uint8_t base_cycles = 0;
};

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
    using AddressModeFunction = bool(CPU::*)();
    using InstructionTable = std::array<Instruction, 256>;

    CPU(Bus* bus);

    void reset();
    void interrupt_request();
    void force_interrupt_request();
    void step();
    [[nodiscard]] bool last_instruction_complete() const;
    void set_program_counter(std::uint16_t address) { program_counter = address; }
    std::uint8_t get_opcode() const { return opcode; }
    std::uint16_t get_program_counter() const { return program_counter; }
    std::uint16_t get_clock() const { return clock_count; }

    static constexpr std::uint16_t stack_address = 0x0100;

private:
    bool get_flag(StatusRegisterFlags flag);
    void set_flag(StatusRegisterFlags flag, bool value);
    void set_flag_true(StatusRegisterFlags flag);
    void set_flag_false(StatusRegisterFlags flag);

    std::uint8_t read_from_memory(std::uint16_t address) const;
    void write_to_memory(std::uint16_t address, std::uint8_t data) const;

    //The read location could be a memory address or part of the instruction
    template <AddressModeFunction AddressMode>
    std::uint8_t fetch();

    [[nodiscard]] constexpr InstructionTable generate_instruction_table() const;

    // clang-format off
    template<AddressModeFunction AddressMode> bool instruction_adc();
    template<AddressModeFunction AddressMode> bool instruction_and();
    template<AddressModeFunction AddressMode> bool instruction_asl();
    template<AddressModeFunction AddressMode> bool instruction_bcc();
    template<AddressModeFunction AddressMode> bool instruction_bcs();
    template<AddressModeFunction AddressMode> bool instruction_beq();
    template<AddressModeFunction AddressMode> bool instruction_bit();
    template<AddressModeFunction AddressMode> bool instruction_bmi();
    template<AddressModeFunction AddressMode> bool instruction_bne();
    template<AddressModeFunction AddressMode> bool instruction_bpl();
    template<AddressModeFunction AddressMode> bool instruction_brk();
    template<AddressModeFunction AddressMode> bool instruction_bvc();
    template<AddressModeFunction AddressMode> bool instruction_bvs();
    template<AddressModeFunction AddressMode> bool instruction_clc();
    template<AddressModeFunction AddressMode> bool instruction_cld();
    template<AddressModeFunction AddressMode> bool instruction_cli();
    template<AddressModeFunction AddressMode> bool instruction_clv();
    template<AddressModeFunction AddressMode> bool instruction_cmp();
    template<AddressModeFunction AddressMode> bool instruction_cpx();
    template<AddressModeFunction AddressMode> bool instruction_cpy();
    template<AddressModeFunction AddressMode> bool instruction_dec();
    template<AddressModeFunction AddressMode> bool instruction_dex();
    template<AddressModeFunction AddressMode> bool instruction_dey();
    template<AddressModeFunction AddressMode> bool instruction_eor();
    template<AddressModeFunction AddressMode> bool instruction_inc();
    template<AddressModeFunction AddressMode> bool instruction_inx();
    template<AddressModeFunction AddressMode> bool instruction_iny();
    template<AddressModeFunction AddressMode> bool instruction_jmp();
    template<AddressModeFunction AddressMode> bool instruction_jsr();
    template<AddressModeFunction AddressMode> bool instruction_lda();
    template<AddressModeFunction AddressMode> bool instruction_ldx();
    template<AddressModeFunction AddressMode> bool instruction_ldy();
    template<AddressModeFunction AddressMode> bool instruction_lsr();
    template<AddressModeFunction AddressMode> bool instruction_nop();
    template<AddressModeFunction AddressMode> bool instruction_ora();
    template<AddressModeFunction AddressMode> bool instruction_pha();
    template<AddressModeFunction AddressMode> bool instruction_php();
    template<AddressModeFunction AddressMode> bool instruction_pla();
    template<AddressModeFunction AddressMode> bool instruction_plp();
    template<AddressModeFunction AddressMode> bool instruction_rol();
    template<AddressModeFunction AddressMode> bool instruction_ror();
    template<AddressModeFunction AddressMode> bool instruction_rti();
    template<AddressModeFunction AddressMode> bool instruction_rts();
    template<AddressModeFunction AddressMode> bool instruction_sbc();
    template<AddressModeFunction AddressMode> bool instruction_sec();
    template<AddressModeFunction AddressMode> bool instruction_sed();
    template<AddressModeFunction AddressMode> bool instruction_sei();
    template<AddressModeFunction AddressMode> bool instruction_sta();
    template<AddressModeFunction AddressMode> bool instruction_stx();
    template<AddressModeFunction AddressMode> bool instruction_sty();
    template<AddressModeFunction AddressMode> bool instruction_tax();
    template<AddressModeFunction AddressMode> bool instruction_tay();
    template<AddressModeFunction AddressMode> bool instruction_tsx();
    template<AddressModeFunction AddressMode> bool instruction_txa();
    template<AddressModeFunction AddressMode> bool instruction_txs();
    template<AddressModeFunction AddressMode> bool instruction_tya();

    template <AddressModeFunction AddressMode, StatusRegisterFlags flag, bool set> bool instruction_branch();
    template <AddressModeFunction AddressMode> bool instruction_modify_register(std::uint8_t& target_register, std::int8_t value);
    template <AddressModeFunction AddressMode> bool instruction_transfer(std::uint8_t& source, std::uint8_t& target);
    template <AddressModeFunction AddressMode> bool instruction_load_register(std::uint8_t& target_register);
    template <AddressModeFunction AddressMode> bool instruction_compare(std::uint8_t& target_register);
    // clang-format on

    bool address_mode_implied();
    bool address_mode_immidiate();
    bool address_mode_accumulator();
    bool address_mode_relative();
    bool address_mode_zero_page();
    bool address_mode_zero_page_y();
    bool address_mode_zero_page_x();
    bool address_mode_absolute();
    bool address_mode_absolute_x();
    bool address_mode_absolute_y();
    bool address_mode_indirect();
    bool address_mode_indirect_x();
    bool address_mode_indirect_y();

    bool unofficial_opcode();

    Bus* bus = nullptr;

    std::uint8_t register_accumulator = 0;
    std::uint8_t register_x = 0;
    std::uint8_t register_y = 0;
    std::uint8_t register_status = 0;
    //points to a memory address that is incremented/decremented when pushing/pulling from the stack
    std::uint8_t stack_pointer = 0;
    //Stores the address of the next program byte
    std::uint16_t program_counter = 0;

    std::uint8_t fetched = 0;          //Represents the working input value to the ALU
    std::uint16_t address_absolute = 0;    //All used memory addresses end up in here
    std::uint16_t address_relative = 0;    //Represents the absolute address following a branch
    std::uint8_t opcode = 0;               //Instruction byte
    std::uint8_t remaining_cycles = 0; //For the last executed instruction
    std::uint32_t clock_count = 0;     //Number of clock cycles executed

    const InstructionTable instruction_table;
    std::shared_ptr<spdlog::logger> dissassembly_logger;
};

template <CPU::AddressModeFunction AddressMode>
std::uint8_t CPU::fetch()
{
    if constexpr (AddressMode != &CPU::address_mode_implied)
        fetched = read_from_memory(address_absolute);

    return fetched;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_adc()
{
    constexpr bool ignore_crossed_page_boundary = false;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();

    const std::uint16_t add = static_cast<std::uint16_t>(register_accumulator) + static_cast<std::uint16_t>(fetched) + static_cast<std::uint16_t>(get_flag(StatusRegisterFlags::Carry));
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

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_and()
{
    constexpr bool ignore_crossed_page_boundary = false;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();
    register_accumulator = register_accumulator & fetched;
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);
    return crossed_page_boundary & !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_asl()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();
    const std::uint16_t result = static_cast<std::uint16_t>(fetched) << 1;
    const bool carry = (result & 0xFF00) > 0;
    set_flag(StatusRegisterFlags::Carry, carry);
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);

    if constexpr (AddressMode == &CPU::address_mode_implied)
        register_accumulator = result & 0x00FF;
    else
        write_to_memory(address_absolute, result & 0x00FF);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode, StatusRegisterFlags flag, bool set>
bool CPU::instruction_branch()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    if (get_flag(flag) == set)
        return crossed_page_boundary && !ignore_crossed_page_boundary;

    //branching costs a cycle
    clock_count++;

    address_absolute = program_counter + address_relative;
    //don't shift, faster
    const std::uint16_t address_absolute_page = address_absolute & 0xFF00;
    const std::uint16_t program_counter_page = program_counter & 0xFF00;

    //different page costs a cycle
    if (address_absolute_page != program_counter_page)
        clock_count++;

    program_counter = address_absolute;
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_bcc()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Carry, false>();
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_bcs()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Carry, true>();
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_beq()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Zero, true>();
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_bit()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();

    const std::uint16_t result = register_accumulator & fetched;

    const bool byte_is_zero = (result & 0x00FF) == 0;
    const bool overflow = fetched & (1 << 6);
    const bool negative = fetched & (1 << 7);
    set_flag(StatusRegisterFlags::Zero, byte_is_zero);
    set_flag(StatusRegisterFlags::Overflow, overflow);
    set_flag(StatusRegisterFlags::Negative, negative);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_bmi()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Negative, true>();
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_bne()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Zero, false>();
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_bpl()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Negative, false>();
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_brk()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    program_counter++;

    set_flag_true(StatusRegisterFlags::DisableInterrupts);

    const std::uint8_t program_counter_low_byte = program_counter & 0x00FF;
    const std::uint8_t program_counter_high_byte = (program_counter >> 8) & 0x00FF;
    write_to_memory(stack_address + stack_pointer, program_counter_high_byte);
    stack_pointer++;
    write_to_memory(stack_address + stack_pointer, program_counter_low_byte);
    stack_pointer++;

    set_flag_true(StatusRegisterFlags::Break);
    write_to_memory(stack_address + stack_pointer, register_status);
    stack_pointer--;
    set_flag_false(StatusRegisterFlags::Break);

    //todo: magic address
    const std::uint16_t low_byte = read_from_memory(0xFFFE);
    std::uint16_t high_byte = read_from_memory(0xFFFF);
    high_byte = high_byte << 8;
    program_counter = high_byte | low_byte;

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_bvc()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Overflow, false>();
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_bvs()
{
    return instruction_branch<AddressMode, StatusRegisterFlags::Overflow, true>();
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_clc()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    set_flag_false(StatusRegisterFlags::Carry);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_cld()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    set_flag_false(StatusRegisterFlags::DecimalMode);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_cli()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    set_flag_false(StatusRegisterFlags::DisableInterrupts);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_clv()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    set_flag_false(StatusRegisterFlags::Overflow);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_compare(std::uint8_t& target_register)
{
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();

    const std::uint16_t result = static_cast<std::uint16_t>(target_register) - static_cast<std::uint16_t>(fetched);
    set_flag(StatusRegisterFlags::Carry, target_register >= fetched);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0x0000);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);

    return crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_cmp()
{
    constexpr bool ignore_crossed_page_boundary = AddressMode != &CPU::address_mode_indirect_y;
    return instruction_compare<AddressMode>(register_accumulator) && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_cpx()
{
    constexpr bool ignore_crossed_page_boundary = true;
    return instruction_compare<AddressMode>(register_x) && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_cpy()
{
    constexpr bool ignore_crossed_page_boundary = true;
    return instruction_compare<AddressMode>(register_y) && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_dec()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();

    const std::uint16_t result = fetched - 1;
    write_to_memory(address_absolute, result & 0x00FF);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_modify_register(std::uint8_t& target_register, std::int8_t value)
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    target_register += value;
    set_flag(StatusRegisterFlags::Zero, target_register == 0);
    set_flag(StatusRegisterFlags::Negative, target_register & 0x80);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_dex()
{
    return instruction_modify_register<AddressMode>(register_x, -1);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_dey()
{
    return instruction_modify_register<AddressMode>(register_y, -1);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_eor()
{
    constexpr bool ignore_crossed_page_boundary = false;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();

    register_accumulator = register_accumulator ^ fetched;
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_inc()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();

    const std::uint16_t result = fetched + 1;
    write_to_memory(address_absolute, result & 0x00FF);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, result & 0x80);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_inx()
{
    return instruction_modify_register<AddressMode>(register_x, 1);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_iny()
{
    return instruction_modify_register<AddressMode>(register_y, 1);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_jmp()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    program_counter = address_absolute;
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_jsr()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    program_counter--;

    const std::uint8_t program_counter_low_byte = program_counter & 0x00FF;
    const std::uint8_t program_counter_high_byte = (program_counter >> 8) & 0x00FF;
    write_to_memory(stack_address + stack_pointer, program_counter_high_byte);
    stack_pointer--;
    write_to_memory(stack_address + stack_pointer, program_counter_low_byte);
    stack_pointer--;

    program_counter = address_absolute;

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_load_register(std::uint8_t& target_register)
{
    constexpr bool ignore_crossed_page_boundary = false;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();
    target_register = fetched;
    set_flag(StatusRegisterFlags::Zero, target_register == 0);
    set_flag(StatusRegisterFlags::Negative, target_register & 0x80);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

//todo: do this within the instruction table instead? or is that too noisy
template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_lda()
{
    return instruction_load_register<AddressMode>(register_accumulator);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_ldx()
{
    return instruction_load_register<AddressMode>(register_x);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_ldy()
{
    return instruction_load_register<AddressMode>(register_y);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_lsr()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();
    set_flag(StatusRegisterFlags::Carry, fetched & 0x0001);

    const std::uint16_t result = fetched >> 1;

    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);

    if constexpr (AddressMode == &CPU::address_mode_implied)
        register_accumulator = result & 0x00FF;
    else
        write_to_memory(address_absolute, result & 0x00FF);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

//todo: make better
template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_nop()
{
    constexpr bool ignore_crossed_page_boundary = false;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    switch (opcode)
    {
    case 0x1C:
    case 0x3C:
    case 0x5C:
    case 0x7C:
    case 0xDC:
    case 0xFC:
        return crossed_page_boundary && !ignore_crossed_page_boundary;
    }

    return false;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_ora()
{
    constexpr bool ignore_crossed_page_boundary = false;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();
    register_accumulator |= fetched;
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_pha()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    write_to_memory(stack_address + stack_pointer, register_accumulator);
    stack_pointer--;

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_php()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    write_to_memory(stack_address + stack_pointer, register_status | static_cast<std::uint8_t>(StatusRegisterFlags::Break) | static_cast<std::uint8_t>(StatusRegisterFlags::Unused));
    set_flag_false(StatusRegisterFlags::Break);
    set_flag_false(StatusRegisterFlags::Unused);
    stack_pointer--;

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_pla()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    stack_pointer++;
    register_accumulator = read_from_memory(stack_address + stack_pointer);
    set_flag(StatusRegisterFlags::Zero, register_accumulator == 0);
    set_flag(StatusRegisterFlags::Negative, register_accumulator & 0x80);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_plp()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    stack_pointer++;
    register_status = read_from_memory(stack_address + stack_pointer);
    //todo: https://wiki.nesdev.com/w/index.php/Status_flags Two instructions (PLP and RTI) pull a byte from the stack and set all the flags. They ignore bits 5 and 4. 
    //nestest doesn't seem to ignore bit 4
    set_flag_false(StatusRegisterFlags::Break);
    set_flag_true(StatusRegisterFlags::Unused);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_rol()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();

    const std::uint16_t result = static_cast<std::uint16_t>(fetched << 1) | static_cast<std::uint16_t>(get_flag(StatusRegisterFlags::Carry));
    set_flag(StatusRegisterFlags::Carry, result & 0xFF00);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, (result & 0x0080) == 0);
    if constexpr (AddressMode == &CPU::address_mode_implied)
        register_accumulator = result & 0x00FF;
    else
        write_to_memory(address_absolute, result & 0x00FF);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_ror()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();

    const std::uint16_t result = static_cast<std::uint16_t>(static_cast<std::uint16_t>(get_flag(StatusRegisterFlags::Carry)) << 7) | static_cast<std::uint16_t>(fetched >> 1);
    set_flag(StatusRegisterFlags::Carry, fetched & 0x01);
    set_flag(StatusRegisterFlags::Zero, (result & 0x00FF) == 0);
    set_flag(StatusRegisterFlags::Negative, result & 0x0080);
    if constexpr (AddressMode == &CPU::address_mode_implied)
        register_accumulator = result & 0x00FF;
    else
        write_to_memory(address_absolute, result & 0x00FF);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_rti()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    stack_pointer++;
    register_status = read_from_memory(stack_address + stack_pointer);
    //todo: https://wiki.nesdev.com/w/index.php/Status_flags Two instructions (PLP and RTI) pull a byte from the stack and set all the flags. They ignore bits 5 and 4.
    //nestest doesn't seem to ignore bit 4
    set_flag_false(StatusRegisterFlags::Break);
    set_flag_false(StatusRegisterFlags::Unused);

    stack_pointer++;
    program_counter = static_cast<std::uint16_t>(read_from_memory(stack_address + stack_pointer));
    stack_pointer++;
    program_counter |= static_cast<std::uint16_t>(read_from_memory(stack_address + stack_pointer)) << 8;

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_rts()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    stack_pointer++;
    std::uint16_t address = stack_address + stack_pointer;
    program_counter = static_cast<std::uint16_t>(read_from_memory(address));
    stack_pointer++;
    address = stack_address + stack_pointer;
    program_counter |= static_cast<std::uint16_t>(read_from_memory(address)) << 8;

    program_counter++;
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_sbc()
{
    constexpr bool ignore_crossed_page_boundary = false;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    fetch<AddressMode>();
    const std::uint16_t result = static_cast<std::uint16_t>(fetched) & 0x00FF;

    //same as ADC
    const std::uint16_t add = static_cast<std::uint16_t>(register_accumulator) + result + static_cast<std::uint16_t>(get_flag(StatusRegisterFlags::Carry));
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
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_sec()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    set_flag_true(StatusRegisterFlags::Carry);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_sed()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    set_flag_true(StatusRegisterFlags::DecimalMode);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_sei()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    set_flag_true(StatusRegisterFlags::DisableInterrupts);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_sta()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    write_to_memory(address_absolute, register_accumulator);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_stx()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    write_to_memory(address_absolute, register_x);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_sty()
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);
    write_to_memory(address_absolute, register_y);
    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_transfer(std::uint8_t& source, std::uint8_t& target)
{
    constexpr bool ignore_crossed_page_boundary = true;
    const bool crossed_page_boundary = std::invoke(AddressMode, *this);

    target = source;
    set_flag(StatusRegisterFlags::Zero, target == 0);
    set_flag(StatusRegisterFlags::Negative, target & 0x80);

    return crossed_page_boundary && !ignore_crossed_page_boundary;
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_tax()
{
    return instruction_transfer<AddressMode>(register_accumulator, register_x);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_tay()
{
    return instruction_transfer<AddressMode>(register_accumulator, register_y);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_tsx()
{
    return instruction_transfer<AddressMode>(stack_pointer, register_x);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_txa()
{
    return instruction_transfer<AddressMode>(register_x, register_accumulator);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_txs()
{
    return instruction_transfer<AddressMode>(register_x, stack_pointer);
}

template <CPU::AddressModeFunction AddressMode>
bool CPU::instruction_tya()
{
    return instruction_transfer<AddressMode>(register_y, register_accumulator);
}