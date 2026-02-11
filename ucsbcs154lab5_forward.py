# ucsbcs154lab5
# All Rights Reserved
# Copyright (c) 2022 University of California Santa Barbara
# Distribution Prohibited

import pyrtl


def ucsbcs154lab5_alu(a, b, ALU_op):
    # Based on the given 'op', return the proper signals as outputs
    ALU_out = pyrtl.WireVector(bitwidth=32)
    with pyrtl.conditional_assignment:
        with ALU_op == pyrtl.Const(0x0):
            # ADD
            ALU_out |= a + b
        with ALU_op == pyrtl.Const(0x1):
            # AND
            ALU_out |= a & b
        with ALU_op == pyrtl.Const(0x2):
            # OR
            ALU_out |= a | b
        with ALU_op == pyrtl.Const(0x3):
            # SLT
            ALU_out |= pyrtl.signed_lt(a, b)
        with ALU_op == pyrtl.Const(0x4):
            # SUB
            ALU_out |= a - b
        with ALU_op == pyrtl.Const(0x5):
            # LU
            ALU_out |= pyrtl.concat(b, pyrtl.Const(0, bitwidth=16))

    zero = (ALU_out == 0)

    return ALU_out, zero

# ----- Does control signal based on func code -----
def ucsbcs154lab5_control(op, func):
    # ADD, AND, ADDI, LUI, ORI, SLT, LW, SW, BEQ
    control_out = pyrtl.WireVector(bitwidth=10, name='control_out')
    with pyrtl.conditional_assignment:
        with op == pyrtl.Const(0x0):
            with func == pyrtl.Const(0x20):
                # ADD
                control_out |= pyrtl.Const(0x280)
            with func == pyrtl.Const(0x24):
                # AND
                control_out |= pyrtl.Const(0x281)
            with func == pyrtl.Const(0x2A):
                # SLT
                control_out |= pyrtl.Const(0x283)
        with op == pyrtl.Const(0x8):
            # ADDI
            control_out |= pyrtl.Const(0x0A0)
        with op == pyrtl.Const(0xF):
            # LUI
            control_out |= pyrtl.Const(0x0A5)
        with op == pyrtl.Const(0xD):
            # ORI
            control_out |= pyrtl.Const(0x0C2)
        with op == pyrtl.Const(0x23):
            # LW
            control_out |= pyrtl.Const(0x0A8)
        with op == pyrtl.Const(0x2B):
            # SW
            control_out |= pyrtl.Const(0x030)
        with op == pyrtl.Const(0x4):
            # BEQ
            control_out |= pyrtl.Const(0x104)

    # ===== Slices Control =====

    # ----- EX -----
    reg_dest = control_out[9]
    ALU_op = control_out[:3]
    ALU_src = control_out[5:7]

    # ----- MEM -----
    branch = control_out[8]
    mem_write = control_out[4]
    
    # ----- WB -----
    reg_write = control_out[7]    
    mem_to_reg = control_out[3]


    return reg_dest, branch, reg_write, ALU_src, mem_write, mem_to_reg, ALU_op


# registers/memory

# ----- Register Files -----
rf = pyrtl.MemBlock(bitwidth=32, addrwidth=32, name='rf', max_read_ports=3, asynchronous=True)

# ----- Instruction Memory -----
i_mem = pyrtl.MemBlock(bitwidth=32, addrwidth=32, name='i_mem', asynchronous=True)

# ----- Data Memory -----
d_mem = pyrtl.MemBlock(bitwidth=32, addrwidth=32, name='d_mem', asynchronous=True)

# ----- Program Counter -----
pc = pyrtl.Register(bitwidth=32, name='pc', reset_value=(2**32)-4)


# ----- PIPELINE_REGISTERS  -----

# ==================== FETCH_DECODE  ====================
# ==================== FETCH_DECODE  ====================


instr_fd = pyrtl.Register(bitwidth=32, name='instr_fd')
pc_plus_4_fd = pyrtl.Register(bitwidth=32, name='pc_plus_4_fd')


# ==================== DECODE_EXECUTE  ====================
# ==================== DECODE_EXECUTE  ====================

# ----- DX_MEANS_INFORMATION_DECODE/EXECUTE_REGISTER -----

# ----- EX -----
reg_dest_dx = pyrtl.Register(bitwidth=1, name='reg_dest_dx')
ALU_op_dx = pyrtl.Register(bitwidth=3, name='ALU_op_dx')
ALU_src_dx = pyrtl.Register(bitwidth=2, name='ALU_src_dx')

# ----- MEM -----
branch_dx = pyrtl.Register(bitwidth=1, name='branch_dx')
mem_write_dx = pyrtl.Register(bitwidth=1, name='mem_write_dx')

# ----- WB -----
reg_write_dx = pyrtl.Register(bitwidth=1, name='reg_write_dx')
mem_to_reg_dx = pyrtl.Register(bitwidth=1, name='mem_to_reg_dx')

# ----- RF_INPUT -----
imm_zero_dx = pyrtl.Register(bitwidth=32, name='imm_zero_dx')
imm_sign_dx = pyrtl.Register(bitwidth=32, name='imm_sign_dx')
rd_dx = pyrtl.Register(bitwidth=32, name='rd_dx')
rt_dx = pyrtl.Register(bitwidth=32, name='rt_dx')
# ====== ADDED THIS ======
rs_dx = pyrtl.Register(bitwidth=32, name = 'rs_dx')
# ====== END OF ADDED THIS ======

# ----- RF_OUTPUT -----
rf_data_1_dx = pyrtl.Register(bitwidth=32, name='rf_data_1_dx')
rf_data_2_dx = pyrtl.Register(bitwidth=32, name='rf_data_2_dx')

# ----- NEXT_INSTR_LOGIC -----
branch_offset_dx = pyrtl.Register(bitwidth=32, name='branch_offset_dx')
pc_plus_4_dx = pyrtl.Register(bitwidth=32, name='pc_plus_4_dx')


# ==================== EXECUTE_MEMORY  ====================
# ==================== EXECUTE_MEMORY  ====================

# ----- MEM -----
to_branch_xm = pyrtl.Register(bitwidth=1, name='to_branch_xm')
mem_write_xm = pyrtl.Register(bitwidth=1, name='mem_write_xm')


# ----- WB -----
reg_write_xm = pyrtl.Register(bitwidth=1, name='reg_write_xm')
mem_to_reg_xm = pyrtl.Register(bitwidth=1, name='mem_to_reg_xm')

# ----- ALU_INPUT -----
rd_xm = pyrtl.Register(bitwidth=32, name='rd_xm')
rf_data_2_xm = pyrtl.Register(bitwidth=32, name='rf_data_2_xm')

""" # katie is adding this
rt_xm = pyrtl.Register(bitwidth=32, name='rt_xm') """

# ----- ALU_OUTPUT -----
ALU_result_xm = pyrtl.Register(bitwidth=32, name='ALU_result_xm')
branch_pc_xm = pyrtl.Register(bitwidth=32, name='branch_pc_xm')


# ==================== MEMORY_WRITEBACK  ====================
# ==================== MEMORY_WRITEBACK  ====================

# ----- WB -----
reg_write_mw = pyrtl.Register(bitwidth=1, name='reg_write_mw')
mem_to_reg_mw = pyrtl.Register(bitwidth=1, name='mem_to_reg_mw')

# ---- MUX_INPUT ----
ALU_result_mw = pyrtl.Register(bitwidth=32, name='ALU_result_mw') 
d_mem_read_mw = pyrtl.Register(bitwidth=32, name='d_mem_read_mw')

# ---- MUX_OUTPUT ----
rd_mw = pyrtl.Register(bitwidth=32, name='rd_mw')

# ==================== END OF THAT STUFF  ====================

# ==================== BEGIN_CYCLEING_LOGIC  ====================

# hazard signals
stall = pyrtl.WireVector(bitwidth=1, name='stall')
to_branch_x = pyrtl.WireVector(bitwidth=1, name='to_branch_x')
rd_x = pyrtl.WireVector(bitwidth=32, name='rd_x')


# fetch
pc_next = pyrtl.WireVector(bitwidth=32, name='pc_next')
with pyrtl.conditional_assignment:
    with stall:
        pc_next |= pc
    with to_branch_xm:
        pc_next |= branch_pc_xm
    with pyrtl.otherwise:
        pc_next |= pc + 4

instr_f = i_mem[pc_next[2:]]
instr_f.name = 'instr_f'

# ==================== (REG->DECODE)  ====================

# fetch REGISTERS -> decode
pc.next <<= pc_next
pc_plus_4_fd.next <<= pc_next + 4
instr_fd.next <<= instr_f

# decode
op_d = instr_fd[26:]
rs_d = instr_fd[21:26]
rt_d = instr_fd[16:21]
rd_d = instr_fd[11:16]
sh_d = instr_fd[6:11]
func_d = instr_fd[:6]
imm_d = instr_fd[:16]

reg_dest_d, branch_d, reg_write_d, ALU_src_d, mem_write_d, mem_to_reg_d, ALU_op_d = \
    ucsbcs154lab5_control(op_d, func_d)
    # ----- THAT FUNCTION does control signal based on func and op code -----

  # extra logic because pyrtl mem writes are always posedge synchronous
rf_write_data_w = pyrtl.WireVector(bitwidth=32, name='rf_write_data_w')
rf_write_enable_w = pyrtl.WireVector(bitwidth=1)
wrote_rs_d = rf_write_enable_w & (rs_d==rd_mw)
wrote_rt_d = rf_write_enable_w & (rt_d==rd_mw)
rf_data_1_d = pyrtl.select(wrote_rs_d, rf_write_data_w, rf[rs_d])
rf_data_2_d = pyrtl.select(wrote_rt_d, rf_write_data_w, rf[rt_d])

imm_zero_d = imm_d.zero_extended(bitwidth=32)
imm_sign_d = imm_d.sign_extended(bitwidth=32)

branch_offset_d = pyrtl.shift_left_arithmetic(imm_sign_d, 2)


# hazard
    # stall if execute has LW, and decode has hazard
stall <<= mem_to_reg_dx & (rd_x!=0) & ((rs_d==rd_x) | (rt_d==rd_x))
    # flush "d->x stage" on stalls and branch miss
flush_dx_stage = stall | to_branch_x | to_branch_xm

# ==================== REG->EXECUTE  ====================

# decode -> execute
with pyrtl.conditional_assignment:
    with flush_dx_stage: # FLUSH_ACTIVATED
        rf_data_1_dx.next |= 0
        rf_data_2_dx.next |= 0
        imm_zero_dx.next |= 0
        imm_sign_dx.next |= 0
        ALU_op_dx.next |= 0
        branch_offset_dx.next |= 0
        pc_plus_4_dx.next |= 0
        branch_dx.next |= 0
        ALU_src_dx.next |= 0
        mem_write_dx.next |= 0
        mem_to_reg_dx.next |= 0
        reg_write_dx.next |= 0
        reg_dest_dx.next |= 0
        rd_dx.next |= 0
        rt_dx.next |= 0
        # ====== ADDED THIS ======
        rs_dx.next |= 0
        # ====== END OF ADDED THIS ======
        
    with pyrtl.otherwise: # FLUSH_INACTIVE
        rf_data_1_dx.next |= rf_data_1_d
        rf_data_2_dx.next |= rf_data_2_d
        imm_zero_dx.next |= imm_zero_d
        imm_sign_dx.next |= imm_sign_d
        ALU_op_dx.next |= ALU_op_d
        branch_offset_dx.next |= branch_offset_d
        pc_plus_4_dx.next |= pc_plus_4_fd
        branch_dx.next |= branch_d
        ALU_src_dx.next |= ALU_src_d
        mem_write_dx.next |= mem_write_d
        mem_to_reg_dx.next |= mem_to_reg_d
        reg_write_dx.next |= reg_write_d
        reg_dest_dx.next |= reg_dest_d
        rd_dx.next |= rd_d
        rt_dx.next |= rt_d
        # ====== ADDED THIS ======
        rs_dx.next |= rs_d
        # ====== END OF ADDED THIS ======

# execute
    # pass rt into rd if I type !!!
rd_x <<= pyrtl.select(reg_dest_dx, rd_dx, rt_dx)

rf_data_1_x = pyrtl.WireVector(bitwidth=32)
rf_data_1_x <<= rf_data_1_dx

rf_data_2_x = pyrtl.WireVector(bitwidth=32)
rf_data_2_x <<= rf_data_2_dx


# ====== TODO: FIXME: HACK: NOTE: I MUST DO SOME CHANGE HERE ======

# ====== FORWARDING UNIT ======
# ----- Declare wire vectors -----
forward_A = pyrtl.WireVector(bitwidth = 2, name = "forward_A") # 00, 01, 10
forward_B = pyrtl.WireVector(bitwidth = 2, name = "forward_B") # 00, 01, 10

# ----- Assign wire vectors -----
# Forward_A
with pyrtl.conditional_assignment:
    with (reg_write_mw 
          & (rd_mw != 0) 
          & ~(reg_write_xm 
              & (rd_xm != 0) 
              & (rd_xm == rs_dx))    # if it is an ex haz, but also mem haz, then go with mem
          & (rd_mw == rs_dx)): # MEM hazard
        forward_A |= 0b1
    with (reg_write_xm 
          & (rd_xm != 0) 
          & (rd_xm == rs_dx)) : # EX hazard
        forward_A |= 0b10
    with pyrtl.otherwise:
        forward_A |= 0b0
        
# Forward_B
with pyrtl.conditional_assignment:
    with (reg_write_mw 
          & (rd_mw != 0) 
          & ~(reg_write_xm 
              & (rd_xm != 0) 
              & (rd_xm == rt_dx))  # if it is an ex haz, but also mem haz, then go with mem
          & (rd_mw == rt_dx)): # MEM hazard
        forward_B |= 0b1
    with (reg_write_xm 
          & (rd_xm != 0) 
          & (rd_xm == rt_dx)) : # EX hazard
        forward_B |= 0b10
    with pyrtl.otherwise:
        forward_B |= 0b0
""" 
Logic taken from textbook:
EX HAZARD: Forward_A
if (EX/MEM.RegWrite
and (EX/MEM.RegisterRd ≠ 0)
and (EX/MEM.RegisterRd = ID/EX.RegisterRs)) ForwardA = 10 

EX HAZARD: Forward_B
if (EX/MEM.RegWrite
and (EX/MEM.RegisterRd ≠ 0)
and (EX/MEM.RegisterRd = ID/EX.RegisterRt)) ForwardB = 10

MEM HAZARD: Forward_A
if (MEM/WB.RegWrite
and (MEM/WB.RegisterRd ≠ 0)
and not(EX/MEM.RegWrite and (EX/MEM.RegisterRd ≠ 0)
and (EX/MEM.RegisterRd ≠ ID/EX.RegisterRs))
and (MEM/WB.RegisterRd = ID/EX.RegisterRs)) ForwardA = 01

MEM HAZARD: Forward_B
if (MEM/WB.RegWrite
and (MEM/WB.RegisterRd ≠ 0)
and not(EX/MEM.RegWrite and (EX/MEM.RegisterRd ≠ 0)
and (EX/MEM.RegisterRd ≠ ID/EX.RegisterRt))
and (MEM/WB.RegisterRd = ID/EX.RegisterRt)) ForwardB = 01
"""


        
# ====== ALU INPUTS, DOUBLE CHECK THESE ======
ALU_first_x = pyrtl.WireVector(bitwidth=32, name='ALU_first_x')
with pyrtl.conditional_assignment:
    with forward_A == 0b00:
        ALU_first_x |= rf_data_1_x
    with forward_A == 0b01:
        ALU_first_x |= rf_write_data_w
    with forward_A == 0b10: # EX
        ALU_first_x |= ALU_result_xm

ALU_second_reg = pyrtl.WireVector(bitwidth=32, name = "ALU_second_reg")
with pyrtl.conditional_assignment:
    with forward_B == 0b00:
        ALU_second_reg |= rf_data_2_x
    with forward_B == 0b01:
        ALU_second_reg |= rf_write_data_w
    with forward_B == 0b10: # EX
        ALU_second_reg |= ALU_result_xm
ALU_second_x = pyrtl.WireVector(bitwidth=32, name='ALU_second_x')
with pyrtl.conditional_assignment:
    with ALU_src_dx==0:
        ALU_second_x |= ALU_second_reg
    with ALU_src_dx==1:
        ALU_second_x |= imm_sign_dx
    with ALU_src_dx==2:
        ALU_second_x |= imm_zero_dx

ALU_result_x, ALU_zero_x = ucsbcs154lab5_alu(ALU_first_x, ALU_second_x, ALU_op_dx)
branch_ALU_result_x = pyrtl.signed_add(pc_plus_4_dx, branch_offset_dx)
to_branch_x <<= branch_dx & ALU_zero_x

""" 
ForwardA = 00 ID/EX The first ALU operand comes from the register file.
ForwardA = 10 EX/MEM The first ALU operand is forwarded from the prior ALU result.
ForwardA = 01 MEM/WB The first ALU operand is forwarded from data memory or an earlier ALU result.
ForwardB = 00 ID/EX The second ALU operand comes from the register file.
ForwardB = 10 EX/MEM The second ALU operand is forwarded from the prior ALU result.
ForwardB = 01 MEM/WB The second ALU operand is forwarded from data memory or an
earlier ALU result.

"""
# ====== TODO: FIXME: HACK: NOTE: END OF CHANGES THAT MUST BE MADE ======

# ==================== (REG->MEMORY)  ====================

# execute -> memory
ALU_result_xm.next <<= ALU_result_x
branch_pc_xm.next <<= branch_ALU_result_x
to_branch_xm.next <<= to_branch_x
mem_write_xm.next <<= mem_write_dx
mem_to_reg_xm.next <<= mem_to_reg_dx
reg_write_xm.next <<= reg_write_dx
rd_xm.next <<= rd_x
rf_data_2_xm.next <<= ALU_second_reg


# memory
d_mem_address_m = pyrtl.shift_left_arithmetic(ALU_result_xm, 2)
d_mem_read_m = d_mem[d_mem_address_m[2:]]
d_mem[d_mem_address_m[2:]] <<= pyrtl.MemBlock.EnabledWrite(rf_data_2_xm, mem_write_xm)

# ==================== (REG->WRITEBACK)  ====================

# memory -> writeback
mem_to_reg_mw.next <<= mem_to_reg_xm
reg_write_mw.next <<= reg_write_xm
rd_mw.next <<= rd_xm
d_mem_read_mw.next <<= d_mem_read_m
ALU_result_mw.next <<= ALU_result_xm


# writeback MUX
with pyrtl.conditional_assignment:
    with mem_to_reg_mw:
        rf_write_data_w |= d_mem_read_mw
    with pyrtl.otherwise:
        rf_write_data_w |= ALU_result_mw

rf_write_enable_w <<= (rd_mw!=0) & reg_write_mw
rf[rd_mw] <<= pyrtl.MemBlock.EnabledWrite(rf_write_data_w, rf_write_enable_w)



##################### SIMULATION #####################

if __name__ == '__main__':

    # Start a simulation trace
    ucsbcs154lab5_sim_trace = pyrtl.SimulationTrace()

    # Initialize the i_mem with your instructions.
    i_mem_init = {}
    with open('i_mem_init.txt', 'r') as fin:
        i = 0
        for line in fin.readlines():
            i_mem_init[i] = int(line, 16)
            i += 1

    sim = pyrtl.Simulation(tracer=ucsbcs154lab5_sim_trace, memory_value_map={
        i_mem: i_mem_init
    })

    # Run for an arbitrarily large number of cycles.
    for cycle in range(1000):
        sim.step({})

    # Use render_trace() to debug if your code doesn't work.
    # ucsbcs154lab5_sim_trace.render_trace(symbol_len=20)
    # ucsbcs154lab5_sim_trace.print_vcd(open('dump.vcd', 'w'), include_clock=True)

    # You can also print out the register file or memory like so if you want to debug:
    print("Register Files: ", sim.inspect_mem(rf))
    print("Data Mem: ", sim.inspect_mem(d_mem))
