// data path
package cpu

import chisel3._
import chisel3.util._
import chisel3.util.experimental._
import cpu.config._
import chisel3.experimental._
import ALU._
import Control._

class MemInterface(implicit p: Parameters) extends CoreBundle()(p) {
  // rd
  val addr = Input(UInt(log2Ceil(memdepth).W))
  val din = Input(UInt(xlen.W))
  val dout = Output(UInt(xlen.W))
  val we = Input(Bool())
}

@chiselName
class Datapath(implicit val p: Parameters) extends Module with CoreParams {
  val io = IO(new Bundle {
//    val instMEM = Flipped(new MemInterface)
//    val dataMEM = Flipped(new MemInterface)
    val wb_out = Output(UInt(xlen.W))
  })
  // memories
  val imem = SyncReadMem(memdepth, UInt(xlen.W))
  val dmem = SyncReadMem(memdepth, UInt(xlen.W))
  loadMemoryFromFile(imem, "/home/l-b/prj/vlsi/src/main/resources/imem.hex")
  loadMemoryFromFile(dmem, "/home/l-b/prj/vlsi/src/main/resources/dmem.hex")
  //RegFile
  val regfile = Module(new RegFile)
  val alu = p(BUILDALU)(p)
  // Brcond
  val br = p(BUILDBR)(p)
  // immgen
  val imm = p(BUILDIMMGEN)(p)
  // Control
  val ctrl = Module(new Control)

  // IF/ID
  val if_inst = RegInit(Instructions.NOP)
  val if_pc = Reg(UInt(64.W))
  dontTouch(if_inst)
  dontTouch(if_pc)

  // ID/EX
  val id_pc = Reg(UInt(xlen.W))
  val id_inst = Reg(UInt(32.W))
  val id_ALU_op = Reg(UInt(4.W))
  val id_ld_op = Reg(UInt(3.W))
  val id_sd_op = Reg(UInt(2.W))
  val id_A = Reg(UInt(xlen.W))
  val id_B = Reg(UInt(xlen.W))
  val id_rs1 = Reg(UInt(xlen.W))
  val id_rs2 = Reg(UInt(xlen.W))
  val id_rd = Reg(UInt(xlen.W))
  val id_wb_en = Reg(Bool())
  val id_imm = Reg(UInt(xlen.W))
  val id_br_type = Reg(UInt(3.W))
  val ctrl_signal = new Bundle{
    val pc_sel = UInt(2.W)
    val A_sel = UInt(1.W)
    val B_sel = UInt(1.W)
  }
  val id_ctrl_signal = Reg(ctrl_signal)
  dontTouch(id_br_type)
  dontTouch(id_sd_op)
  dontTouch(id_ctrl_signal)
  dontTouch(id_pc)
  dontTouch(id_imm)
  dontTouch(id_inst)
  dontTouch(id_ALU_op)
  dontTouch(id_ld_op)
  dontTouch(id_A)
  dontTouch(id_B)
  dontTouch(id_rs1)
  dontTouch(id_rs2)
  dontTouch(id_rd)
  dontTouch(id_wb_en)

  // EX/MEM
  val ex_inst = Reg(UInt(32.W))
  val ex_rdata2 = Reg(UInt(xlen.W))
  val ex_addr = Reg(UInt(xlen.W))
  val ex_mem_wr = Reg(Bool())
  val ex_rs1 = Reg(UInt(xlen.W))
  val ex_rs2 = Reg(UInt(xlen.W))
  val ex_rd = Reg(UInt(xlen.W))
  val ex_ALU_op = Reg(UInt(4.W))
  val ex_ld_op = Reg(UInt(3.W))
  val ex_wb_en = Reg(Bool())
  val ex_ctrl_signal = Reg(ctrl_signal)
  dontTouch(ex_ctrl_signal)
  dontTouch(ex_inst)
  dontTouch(ex_rdata2)
  dontTouch(ex_addr)
  dontTouch(ex_mem_wr)
  dontTouch(ex_rs1)
  dontTouch(ex_rs2)
  dontTouch(ex_rd)
  dontTouch(ex_ALU_op)
  dontTouch(ex_ld_op)
  dontTouch(ex_wb_en)

  // MEM/WB
  val mem_out = Wire(UInt(xlen.W))
  val mem_rdata2 = Reg(UInt(xlen.W))
  val mem_alu_out = Reg(UInt(xlen.W))
  val mem_rd = Reg(UInt(xlen.W))
  val mem_rs1 = Reg(UInt(xlen.W))
  val mem_rs2 = Reg(UInt(xlen.W))
  val mem_ALU_op = Reg(UInt(4.W))
  val mem_ld_op = Reg(UInt(3.W))
  val mem_wb_en = Reg(Bool())
  val mem_ctrl_signal = Reg(ctrl_signal)
  val mem_inst = Reg(UInt(32.W))
  dontTouch(mem_alu_out)
  dontTouch(mem_ctrl_signal)
  dontTouch(mem_out)
  dontTouch(mem_rdata2)
  dontTouch(mem_rd)
  dontTouch(mem_rs1)
  dontTouch(mem_rs2)
  dontTouch(mem_ALU_op)
  dontTouch(mem_ld_op)
  dontTouch(mem_wb_en)

  // WB
  val wb_rd = Wire(UInt(xlen.W))
  val wb_data = Wire(UInt(xlen.W))

  dontTouch(wb_rd)

  //data forward
  val memA_fw = Wire(Bool())
  val memB_fw = Wire(Bool())
  val wbA_fw = Wire(Bool())
  val wbB_fw = Wire(Bool())
  val wbA_fwLD = Wire(Bool())
  val wbB_fwLD = Wire(Bool())
  dontTouch(memA_fw)
  dontTouch(memB_fw)
  dontTouch(wbA_fw)
  dontTouch(wbB_fw)
  dontTouch(wbA_fwLD)
  dontTouch(wbB_fwLD)

  val stall = Wire(Bool())

  // IF path
  val pc = RegInit(0.U(64.W) - 4.U(64.W))
  pc := Mux(stall, pc,
    Mux(br.io.taken, alu.io.out, pc + 4.U))
  val pc_delay = Reg(UInt(64.W))
  pc_delay := Mux(stall, pc_delay,
    Mux(br.io.taken, "hffff_ffff_ffff_fffc".U, pc))
  // for wait imem
  if_pc := Mux(stall, if_pc, Mux(br.io.taken, "hffff_ffff_ffff_fffc".U, pc_delay))

  val imem_addr = Mux(stall, pc_delay,
    Mux(br.io.taken, "hffff_ffff_ffff_fffc".U, pc))

  if_inst := Mux(stall, if_inst,
    Mux(br.io.taken, Instructions.NOP, imem.read((imem_addr >> 2.U).asUInt())))

  // ID
  val id_tmp_rs1 = WireInit(if_inst(19, 15))
  val id_tmp_rs2 = WireInit(if_inst(24, 20))
  val id_tmp_rd = WireInit(if_inst(11, 7))

  stall := (id_ld_op === LD_LD) & ((id_rd === id_tmp_rs1) | (id_rd === id_tmp_rs2))

  imm.io.inst := if_inst
  imm.io.sel := ctrl.io.imm_sel
  id_sd_op := ctrl.io.st_type
  id_imm := imm.io.out
  id_inst := if_inst
  id_rs1 := id_tmp_rs1
  id_rs2 := id_tmp_rs2
  id_rd := id_tmp_rd
  regfile.io.raddr1 := id_tmp_rs1
  regfile.io.raddr2 := id_tmp_rs2
  id_A := Mux((wb_rd===id_tmp_rs1)&(mem_wb_en===Y), wb_data, regfile.io.rdata1)
  id_B := Mux((wb_rd===id_tmp_rs2)&(mem_wb_en===Y), wb_data, regfile.io.rdata2)
  ctrl.io.inst := if_inst
  when(stall | br.io.taken){
    id_inst := Instructions.NOP
    ctrl.io.inst := Instructions.NOP
    id_rs1 := 0.U
    id_rs2 := 0.U
    id_rd := 0.U
  }

  id_ctrl_signal.pc_sel := ctrl.io.pc_sel
  id_ctrl_signal.A_sel := ctrl.io.A_sel
  id_ctrl_signal.B_sel := ctrl.io.B_sel
  id_br_type := ctrl.io.br_type
  id_ALU_op := ctrl.io.alu_op
  id_ld_op := ctrl.io.ld_type
  id_wb_en := ctrl.io.wb_en
  id_pc := if_pc

  // EX
  ex_wb_en := id_wb_en
  ex_ld_op := id_ld_op
  ex_rs1 := id_rs1
  ex_rs2 := id_rs2
  ex_rd := id_rd
  ex_inst := id_inst
  val Ain = Mux(id_rs1 === 0.U, 0.U, Mux(memA_fw, ex_addr, Mux(
    wbA_fw, mem_alu_out, Mux(
      wbA_fwLD, mem_out, id_A
    ))))
  alu.io.A := Mux(id_ctrl_signal.A_sel === A_RS1, Ain, id_pc)
  val Bin = Mux(id_rs2 === 0.U, 0.U, Mux(memB_fw, ex_addr, Mux(
    wbB_fw, mem_alu_out, Mux(
      wbB_fwLD, mem_out, id_B
    ))))
  alu.io.B := Mux(id_ctrl_signal.B_sel === B_RS2, Bin, id_imm)
  alu.io.alu_op := id_ALU_op
  ex_addr := alu.io.out
  ex_rdata2 := Bin
  ex_mem_wr := id_sd_op.orR()
  ex_ctrl_signal := id_ctrl_signal

  br.io.rs1 := Ain
  br.io.rs2 := Bin
  br.io.br_type := id_br_type

  // MEM
  mem_wb_en := ex_wb_en
  mem_ld_op := ex_ld_op
  mem_rs1 := ex_rs1
  mem_rs2 := ex_rs2
  mem_rd := ex_rd
  mem_inst := ex_inst
  when(ex_mem_wr){
    dmem.write((ex_addr >> 3.U).asUInt(), ex_rdata2)
  }
  mem_alu_out := ex_addr
  mem_out := dmem.read((ex_addr>>3.U).asUInt())
  mem_rdata2 := ex_rdata2
  mem_ctrl_signal := ex_ctrl_signal

  memA_fw := (ex_rd === id_rs1) & (id_rs1 =/= 0.U) & (ex_inst(6, 0) === "b0110011".U)
  memB_fw := (ex_rd === id_rs2) & (id_rs2 =/= 0.U) & (ex_inst(6, 0) === "b0110011".U)

  // WB
  wb_rd := mem_rd
  regfile.io.waddr := wb_rd

  wb_data := Mux(mem_ld_op === LD_LD, mem_out, mem_alu_out)
  regfile.io.wdata := wb_data
  regfile.io.wen := mem_wb_en

  wbA_fw := (mem_rd === id_rs1) & (id_rs1 =/= 0.U) & (mem_inst(6, 0) === "b0110011".U)
  wbB_fw := (mem_rd === id_rs2) & (id_rs2 =/= 0.U) & (mem_inst(6, 0) === "b0110011".U)
  wbA_fwLD := (mem_rd === id_rs1) & (id_rs1 =/= 0.U) & (mem_ld_op === LD_LD)
  wbB_fwLD := (mem_rd === id_rs2) & (id_rs2 =/= 0.U) & (mem_ld_op === LD_LD)


  // test
  io.wb_out := Mux(mem_ld_op === LD_LD, mem_out, mem_rdata2)
}