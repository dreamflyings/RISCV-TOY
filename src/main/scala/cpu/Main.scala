// main
package cpu

import cpu.config._
import chisel3._
import chisel3.util._
import chisel3.experimental._

class DefaultConfig extends Config((site, here, up) => {
  case XLEN     => 64
  case BUILDALU => (p: Parameters) => Module(new ALUSimple()(p))
  case BUILDBR   => (p: Parameters) => Module(new BrCondSimple()(p))
  case BUILDIMMGEN   => (p: Parameters) => Module(new ImmGenWire()(p))
  case MEMDEPTH => 128
})

class Main {

}
