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
    Carry = 1 << 0,
    Zero = 1 << 1,
    DisableInterrupts = 1 << 2,
    DecimalMode = 1 << 3, //Unused
    Break = 1 << 4,
    Unused = 1 << 5,
    Overflow = 1 << 6,
    Negative = 1 << 7,
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

    std::uint8_t read_from_memory(std::uint16_t address);
    void write_to_memory(std::uint16_t address, std::uint8_t data);

    //The read location could be a memory address or part of the instruction
    std::uint8_t fetch();
    
    using InstructionTable = std::array<Instruction, 256>;
    static constexpr InstructionTable generate_instruction_table();


    using AddressModeFunction = std::uint8_t (CPU::*)();
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
    std::uint8_t remaining_cycles;; //For the last executed instruction
    std::uint32_t clock_count = 0; //Number of clock cycles executed

    InstructionTable instruction_table = generate_instruction_table();
};