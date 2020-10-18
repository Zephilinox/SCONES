#pragma once

//STD
#include <cstdint>
#include <array>

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

enum StatusRegisterFlags : std::uint8_t
{
    Invalid = 0,
    Zero = 1 << 0,
    CarryBit = 1 << 2,
    DisableInterrupts = 1 << 3,
    DecimalMode = 1 << 4, //Unused
    Break = 1 << 5,
    Unused = 1 << 6,
    Overflow = 1 << 7,
    Negative = 1 << 8,
};

class Bus
{
    
};

class CPU
{
public:
    CPU(Bus* bus)
        : bus(bus)
    {

    }

    void reset();
    void interrupt_request();
    void force_interrupt_request();
    void step();
    bool last_instruction_complete();

private:
    StatusRegisterFlags get_flag(StatusRegisterFlags flag);
    void set_flag(StatusRegisterFlags flag, bool value);

    Bus* bus = nullptr;
    std::uint8_t read_from_memory(std::uint16_t address);
    void write_to_memory(std::uint16_t address, std::uint8_t data);

    //The read location could be a memory address or part of the instruction
    std::uint8_t fetch();

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
    std::uint8_t remaining_cycles;; //For the last executed instruction
    std::uint32_t clock_count = 0; //Number of clock cycles executed

    using AddressModeFunction = std::uint8_t(CPU::*)();

    using InstructionTable = std::array<Instruction, 256>;
    InstructionTable generate_instruction_table()
    {
        InstructionTable instruction_table{};

        //http://archive.6502.org/datasheets/rockwell_r650x_r651x.pdf
        // clang-format off
        instruction_table[0x00] = Instruction{ &CPU::instruction_break<&CPU::address_mode_implied>,     1, 7 };
        instruction_table[0x01] = Instruction{ &CPU::instruction_ora<&CPU::address_mode_indirect_x>,    2, 6 };
        instruction_table[0x05] = Instruction{ &CPU::instruction_ora<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0x06] = Instruction{ &CPU::instruction_asl<&CPU::address_mode_zero_page>,     2, 5 };
        instruction_table[0x08] = Instruction{ &CPU::instruction_php<&CPU::address_mode_implied>,       1, 3 };
        instruction_table[0x09] = Instruction{ &CPU::instruction_ora<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0x0A] = Instruction{ &CPU::instruction_asl<&CPU::address_mode_accumulator>,   1, 2 };
        instruction_table[0x0D] = Instruction{ &CPU::instruction_ora<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0x0E] = Instruction{ &CPU::instruction_asl<&CPU::address_mode_absolute>,      3, 6 };
        
        instruction_table[0x10] = Instruction{ &CPU::instruction_bpl<&CPU::address_mode_relative>,      2, 2 };
        instruction_table[0x11] = Instruction{ &CPU::instruction_ora<&CPU::address_mode_indirect_y>,    2, 5 };
        instruction_table[0x15] = Instruction{ &CPU::instruction_ora<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0x16] = Instruction{ &CPU::instruction_asl<&CPU::address_mode_zero_page_x>,   2, 6 };
        instruction_table[0x18] = Instruction{ &CPU::instruction_clc<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0x19] = Instruction{ &CPU::instruction_ora<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0x1D] = Instruction{ &CPU::instruction_ora<&CPU::address_mode_absolute_x>,    3, 4 };
        instruction_table[0x1E] = Instruction{ &CPU::instruction_asl<&CPU::address_mode_absolute_x>,    3, 7 };

        instruction_table[0x20] = Instruction{ &CPU::instruction_jsr<&CPU::address_mode_absolute>,      3, 6 };
        instruction_table[0x21] = Instruction{ &CPU::instruction_and<&CPU::address_mode_indirect_x>,    2, 6 };
        instruction_table[0x24] = Instruction{ &CPU::instruction_bit<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0x25] = Instruction{ &CPU::instruction_and<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0x26] = Instruction{ &CPU::instruction_rol<&CPU::address_mode_zero_page>,     2, 5 };
        instruction_table[0x28] = Instruction{ &CPU::instruction_plp<&CPU::address_mode_implied>,       1, 4 };
        instruction_table[0x29] = Instruction{ &CPU::instruction_and<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0x2A] = Instruction{ &CPU::instruction_rol<&CPU::address_mode_accumulator>,   1, 2 };
        instruction_table[0x2C] = Instruction{ &CPU::instruction_bit<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0x2D] = Instruction{ &CPU::instruction_and<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0x2E] = Instruction{ &CPU::instruction_rol<&CPU::address_mode_absolute>,      3, 6 };

        instruction_table[0x30] = Instruction{ &CPU::instruction_bmi<&CPU::address_mode_relative>,      2, 2 };
        instruction_table[0x31] = Instruction{ &CPU::instruction_and<&CPU::address_mode_indirect_y>,    2, 5 };
        instruction_table[0x35] = Instruction{ &CPU::instruction_and<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0x36] = Instruction{ &CPU::instruction_rol<&CPU::address_mode_zero_page_x>,   2, 6 };
        instruction_table[0x38] = Instruction{ &CPU::instruction_sec<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0x39] = Instruction{ &CPU::instruction_and<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0x3D] = Instruction{ &CPU::instruction_and<&CPU::address_mode_absolute_x>,    3, 4 };
        instruction_table[0x3E] = Instruction{ &CPU::instruction_rol<&CPU::address_mode_absolute_x>,    3, 7 };
        
        instruction_table[0x40] = Instruction{ &CPU::instruction_rti<&CPU::address_mode_implied>,       1, 6 };
        instruction_table[0x41] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_indirect_x>,    2, 6 };
        instruction_table[0x45] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0x46] = Instruction{ &CPU::instruction_lsr<&CPU::address_mode_zero_page>,     2, 5 };
        instruction_table[0x48] = Instruction{ &CPU::instruction_pha<&CPU::address_mode_implied>,       1, 3 };
        instruction_table[0x49] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0x4A] = Instruction{ &CPU::instruction_lsr<&CPU::address_mode_accumulator>,   1, 2 };
        instruction_table[0x4C] = Instruction{ &CPU::instruction_jmp<&CPU::address_mode_absolute>,      3, 3 };
        instruction_table[0x4D] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0x4E] = Instruction{ &CPU::instruction_lsr<&CPU::address_mode_absolute>,      3, 6 };
        
        instruction_table[0x50] = Instruction{ &CPU::instruction_bvc<&CPU::address_mode_relative>,      2, 2 };
        instruction_table[0x51] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_indirect_y>,    2, 5 };
        instruction_table[0x55] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0x56] = Instruction{ &CPU::instruction_lsr<&CPU::address_mode_zero_page_x>,   2, 6 };
        instruction_table[0x58] = Instruction{ &CPU::instruction_cli<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0x59] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_absolute_y,     3, 4 };
        instruction_table[0x5D] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_absolute_x,     3, 4 };
        instruction_table[0x5E] = Instruction{ &CPU::instruction_lsr<&CPU::address_mode_absolute_x,     3, 7 };
        
        instruction_table[0x60] = Instruction{ &CPU::instruction_rts<&CPU::address_mode_implied>,       1, 6 };
        instruction_table[0x61] = Instruction{ &CPU::instruction_adc<&CPU::address_mode_indirect_x>,    2, 6 };
        instruction_table[0x65] = Instruction{ &CPU::instruction_adc<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0x66] = Instruction{ &CPU::instruction_ror<&CPU::address_mode_zero_page>,     2, 5 };
        instruction_table[0x68] = Instruction{ &CPU::instruction_pla<&CPU::address_mode_implied>,       1, 4 };
        instruction_table[0x69] = Instruction{ &CPU::instruction_adc<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0x6A] = Instruction{ &CPU::instruction_ror<&CPU::address_mode_accumulator>,   1, 2 };
        instruction_table[0x6C] = Instruction{ &CPU::instruction_jmp<&CPU::address_mode_indirect>,      3, 5 };
        instruction_table[0x6D] = Instruction{ &CPU::instruction_adc<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0x6E] = Instruction{ &CPU::instruction_ror<&CPU::address_mode_absolute>,      3, 6 };

        instruction_table[0x70] = Instruction{ &CPU::instruction_bvs<&CPU::address_mode_relative>,      2, 2 };
        instruction_table[0x71] = Instruction{ &CPU::instruction_adc<&CPU::address_mode_indirect_y>,    2, 5 };
        instruction_table[0x75] = Instruction{ &CPU::instruction_adc<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0x76] = Instruction{ &CPU::instruction_ror<&CPU::address_mode_zero_page_x>,   2, 6 };
        instruction_table[0x78] = Instruction{ &CPU::instruction_sei<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0x79] = Instruction{ &CPU::instruction_adc<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0x7D] = Instruction{ &CPU::instruction_adc<&CPU::address_mode_absolute_x>,    3, 4 };
        instruction_table[0x7E] = Instruction{ &CPU::instruction_ror<&CPU::address_mode_absolute_x>,    3, 7 };

        // clang-format on
        return instruction_table;
    }

    InstructionTable instruction_table = generate_instruction_table();

    template<AddressModeFunction AddressMode> std::uint8_t instruction_break()
    {
        std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

        program_counter++;

        //todo

        return 0;
    }

    template<AddressModeFunction AddressMode> std::uint8_t instruction_ora()
    {
        std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

        program_counter++;

        //todo

        return 0;
    }

    template<AddressModeFunction AddressMode> std::uint8_t instruction_asl()
    {
        std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

        program_counter++;

        //todo

        return 0;
    }

    template<AddressModeFunction AddressMode> std::uint8_t instruction_php()
    {
        std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

        program_counter++;

        //todo

        return 0;
    }

    template<AddressModeFunction AddressMode> std::uint8_t instruction_bpl()
    {
        std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

        program_counter++;

        //todo

        return 0;
    }

    template<AddressModeFunction AddressMode> std::uint8_t instruction_clc()
    {
        std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

        program_counter++;

        //todo

        return 0;
    }

    std::uint8_t address_mode_implied()
    {
        fetched = register_accumulator;
        return 0;
    }

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
};