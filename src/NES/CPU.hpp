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
        instruction_table[0x00] = Instruction{ &CPU::instruction_brk<&CPU::address_mode_implied>,       1, 7 };
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
        instruction_table[0x59] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0x5D] = Instruction{ &CPU::instruction_eor<&CPU::address_mode_absolute_x>,    3, 4 };
        instruction_table[0x5E] = Instruction{ &CPU::instruction_lsr<&CPU::address_mode_absolute_x>,    3, 7 };
        
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
        
        instruction_table[0x81] = Instruction{ &CPU::instruction_sta<&CPU::address_mode_indirect_x>,    2, 6 };
        instruction_table[0x84] = Instruction{ &CPU::instruction_sty<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0x85] = Instruction{ &CPU::instruction_sta<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0x86] = Instruction{ &CPU::instruction_stx<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0x88] = Instruction{ &CPU::instruction_dey<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0x8A] = Instruction{ &CPU::instruction_txa<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0x8C] = Instruction{ &CPU::instruction_sty<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0x8D] = Instruction{ &CPU::instruction_sta<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0x8E] = Instruction{ &CPU::instruction_stx<&CPU::address_mode_absolute>,      3, 4 };

        instruction_table[0x90] = Instruction{ &CPU::instruction_bcc<&CPU::address_mode_relative>,      2, 2 };
        instruction_table[0x91] = Instruction{ &CPU::instruction_sta<&CPU::address_mode_indirect_y>,    2, 6 };
        instruction_table[0x94] = Instruction{ &CPU::instruction_sty<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0x95] = Instruction{ &CPU::instruction_sta<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0x96] = Instruction{ &CPU::instruction_stx<&CPU::address_mode_zero_page_y>,   2, 4 };
        instruction_table[0x98] = Instruction{ &CPU::instruction_tya<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0x99] = Instruction{ &CPU::instruction_sta<&CPU::address_mode_absolute_y>,    3, 5 };
        instruction_table[0x9A] = Instruction{ &CPU::instruction_txs<&CPU::address_mode_relative>,      1, 2 };
        instruction_table[0x9D] = Instruction{ &CPU::instruction_sta<&CPU::address_mode_absolute_x>,    3, 5 };

        instruction_table[0xA0] = Instruction{ &CPU::instruction_ldy<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0xA1] = Instruction{ &CPU::instruction_lda<&CPU::address_mode_indirect_x>,    2, 6 };
        instruction_table[0xA2] = Instruction{ &CPU::instruction_ldx<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0xA4] = Instruction{ &CPU::instruction_ldy<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0xA5] = Instruction{ &CPU::instruction_lda<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0xA6] = Instruction{ &CPU::instruction_ldx<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0xA8] = Instruction{ &CPU::instruction_tay<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xA9] = Instruction{ &CPU::instruction_lda<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0xAA] = Instruction{ &CPU::instruction_tax<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xAC] = Instruction{ &CPU::instruction_ldy<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0xAD] = Instruction{ &CPU::instruction_lda<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0xAE] = Instruction{ &CPU::instruction_ldx<&CPU::address_mode_absolute>,      3, 4 };

        instruction_table[0xB0] = Instruction{ &CPU::instruction_bcs<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0xB1] = Instruction{ &CPU::instruction_lda<&CPU::address_mode_indirect_x>,    2, 5 };
        instruction_table[0xB4] = Instruction{ &CPU::instruction_ldy<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0xB5] = Instruction{ &CPU::instruction_lda<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0xB6] = Instruction{ &CPU::instruction_ldx<&CPU::address_mode_zero_page_y>,   2, 4 };
        instruction_table[0xB8] = Instruction{ &CPU::instruction_clv<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xB9] = Instruction{ &CPU::instruction_lda<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0xBA] = Instruction{ &CPU::instruction_tsx<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xBC] = Instruction{ &CPU::instruction_ldy<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0xBD] = Instruction{ &CPU::instruction_lda<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0xBE] = Instruction{ &CPU::instruction_ldx<&CPU::address_mode_absolute_y>,    3, 4 };

        instruction_table[0xC0] = Instruction{ &CPU::instruction_cpy<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0xC1] = Instruction{ &CPU::instruction_cmp<&CPU::address_mode_indirect_x>,    2, 6 };
        instruction_table[0xC4] = Instruction{ &CPU::instruction_cpy<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0xC5] = Instruction{ &CPU::instruction_cmp<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0xC6] = Instruction{ &CPU::instruction_dec<&CPU::address_mode_zero_page>,     2, 5 };
        instruction_table[0xC8] = Instruction{ &CPU::instruction_iny<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xC9] = Instruction{ &CPU::instruction_cmp<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0xCA] = Instruction{ &CPU::instruction_dex<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xCC] = Instruction{ &CPU::instruction_cpy<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0xCD] = Instruction{ &CPU::instruction_cmp<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0xCE] = Instruction{ &CPU::instruction_dec<&CPU::address_mode_absolute>,      3, 6 };
        
        instruction_table[0xD0] = Instruction{ &CPU::instruction_bne<&CPU::address_mode_relative>,      2, 2 };
        instruction_table[0xD1] = Instruction{ &CPU::instruction_cmp<&CPU::address_mode_indirect_y>,    2, 5 };
        instruction_table[0xD5] = Instruction{ &CPU::instruction_cmp<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0xD6] = Instruction{ &CPU::instruction_dec<&CPU::address_mode_zero_page_x>,   2, 6 };
        instruction_table[0xD8] = Instruction{ &CPU::instruction_cld<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xD9] = Instruction{ &CPU::instruction_cmp<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0xDD] = Instruction{ &CPU::instruction_cmp<&CPU::address_mode_absolute_x>,    3, 4 };
        instruction_table[0xDE] = Instruction{ &CPU::instruction_dec<&CPU::address_mode_absolute_x>,    3, 7 };

        instruction_table[0xE0] = Instruction{ &CPU::instruction_cpx<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0xE1] = Instruction{ &CPU::instruction_sbc<&CPU::address_mode_indirect_x>,    2, 6 };
        instruction_table[0xE4] = Instruction{ &CPU::instruction_cpx<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0xE5] = Instruction{ &CPU::instruction_sbc<&CPU::address_mode_zero_page>,     2, 3 };
        instruction_table[0xE6] = Instruction{ &CPU::instruction_inc<&CPU::address_mode_zero_page>,     2, 5 };
        instruction_table[0xE8] = Instruction{ &CPU::instruction_inx<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xE9] = Instruction{ &CPU::instruction_sbc<&CPU::address_mode_immidiate>,     2, 2 };
        instruction_table[0xEA] = Instruction{ &CPU::instruction_nop<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xEC] = Instruction{ &CPU::instruction_cpx<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0xED] = Instruction{ &CPU::instruction_sbc<&CPU::address_mode_absolute>,      3, 4 };
        instruction_table[0xEE] = Instruction{ &CPU::instruction_inc<&CPU::address_mode_absolute>,      3, 6 };

        instruction_table[0xF0] = Instruction{ &CPU::instruction_beq<&CPU::address_mode_relative>,      2, 2 };
        instruction_table[0xF1] = Instruction{ &CPU::instruction_sbc<&CPU::address_mode_indirect_y>,    2, 5 };
        instruction_table[0xF5] = Instruction{ &CPU::instruction_sbc<&CPU::address_mode_zero_page_x>,   2, 4 };
        instruction_table[0xF6] = Instruction{ &CPU::instruction_inc<&CPU::address_mode_zero_page_x>,   2, 6 };
        instruction_table[0xF8] = Instruction{ &CPU::instruction_sed<&CPU::address_mode_implied>,       1, 2 };
        instruction_table[0xF9] = Instruction{ &CPU::instruction_sbc<&CPU::address_mode_absolute_y>,    3, 4 };
        instruction_table[0xFD] = Instruction{ &CPU::instruction_sbc<&CPU::address_mode_absolute_x>,    3, 4 };
        instruction_table[0xFE] = Instruction{ &CPU::instruction_inc<&CPU::address_mode_absolute_x>,    3, 7 };
        // clang-format on

        return instruction_table;
    }

    InstructionTable instruction_table = generate_instruction_table();

    // clang-format off
    template<AddressModeFunction AddressMode> std::uint8_t instruction_adc()
    {
        std::uint8_t extra_cycles_from_addressing = (*this.*AddressMode)();

        program_counter++;

        //todo

        return 0;
    }

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
    // clang-format on

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