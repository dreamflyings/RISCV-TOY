// test
package cpu

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}

class cpuTests(c: Datapath) extends PeekPokeTester(c) {
  step(200)
}

class cpuTester extends ChiselFlatSpec {
  implicit val conf = new DefaultConfig
  "running with --generate-vcd-output on" should "create a vcd file from your test" in {
    iotesters.Driver.execute(
      Array(
        "--generate-vcd-output", "on",
        "--target-dir", "test_run_dir/make_CPU_vcd",
        "--backend-name", "verilator",
        "--top-name", "make_TB_vcd"
      ), () => new Datapath) {
      c => new cpuTests(c)
    } should be(true)
  }
}