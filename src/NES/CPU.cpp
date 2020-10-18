#include "CPU.hpp"

constexpr CPU::InstructionTable CPU::generate_instruction_table()
{
    InstructionTable instruction_table{};

    for (int i = 0; i < 256; ++i) {
        instruction_table[i] = Instruction{ &CPU::unofficial_opcode, 0, 0 };
    }

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