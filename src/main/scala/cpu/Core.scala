//
package cpu

import chisel3._
import chisel3.util._
import chisel3.experimental._
import cpu.config._

case object XLEN extends Field[Int]

case object MEMDEPTH extends Field[Int]

case object BUILDALU extends Field[Parameters => ALU]

case object BUILDBR extends Field[Parameters => BrCond]

case object BUILDIMMGEN extends Field[Parameters => ImmGen]

abstract trait CoreParams {
  implicit val p: Parameters
  val xlen = p(XLEN)
  val memdepth = p(MEMDEPTH)
}

abstract class CoreBundle(implicit val p: Parameters) extends Bundle with CoreParams

