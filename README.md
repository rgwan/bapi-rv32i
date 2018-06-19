This repository gives you a very mean (a.k.a ba pi) RISC-V RV32I implementation for RISC-V Shanghai Day 2018.

The synthesize result on Anlogic EG4-20 is about 110MHz fmax and 500LUTs + 4/2 M32Ks. Now I'm improving timing and instruction set support. It is not usable yet, but I will let it fully works before Jun.28 2018.

Currently it can execute jump, auipc, op-imm, op-r2r and 32-bit load-store. But compare, branch instruction are not supported yet: I'm working on it.

I wouldn't implement there features:

1. Byte/half word load-store, because it may causes about 100LUTs. I perfer user use software to implement it.

2. Unaligned load-store, same reason as 1.

3. [Maybe] Interrupt controller.

4. [Maybe] Reset logic.

5. [Maybe] RVC decoder.


Jun.19 2018  Zhiyuan Wan 
