# 6809 Playground

Welcome to my 6809 Playground!  This project is about building a general
purpose 8-bit computer based on the Motorola 6809 CPU.  While this is
definitely a retro-computer project, I'm not going to be dogmatic about
it -- modern gadgets that add value to the project will have a home here,
too.

The whole point of this exercise is to learn-by-doing and have fun in the
process.  I have a few ideas about how I want the project to go, but it
may veer this way or that as it progresses.  We'll see!

I invite you to follow along with this [YouTube playlist](https://youtube.com/playlist?list=PL_E3Je1H0xrvq1AOL1zlwj3LmrBiK8FJb).  As each
installment is posted, this README will be updated with additional information
about the that installment, including links to data sheets and a parts list.

If you learn something from this, if you decide to build one of these
yourself, or if it inspires you embark on a similar adventure on your own,
I would love to hear from you!

## 01 - Introduction

In the first installment, I briefly introduce the project and provide some
background.

Video link: [01 - Introduction](https://youtu.be/d_VZdUXh_dU)

## 02 - Clock Generator

In the second installment, I describe the design and implementation of
the clock generator circuitry.

Major components:
* [74ACT163](https://www.ti.com/lit/ds/symlink/cd74act163.pdf) binary counter
* [74ACT109](https://www.ti.com/lit/ds/symlink/cd74act109.pdf) dual J-/K flip-flop
* [Lattice GAL22V10](https://web.mit.edu/6.115/www/document/gal22v10.pdf) or
  [Atmel ATF22V10](https://www.mouser.com/datasheet/2/268/doc0735-1369018.pdf)
  electrically-eraseable programmable logic device

Video link: [02 - Clock Generator](https://youtu.be/Hb_b3B4GNCY)

## 03 - Reset Generator

This the third installment, I describe how the reset generator works.

Major components:
* [Dallas Semiconductor DS1813](https://www.mouser.com/datasheet/2/609/DS1813-3122044.pdf)

Video link: [03 - Reset Generator](https://youtu.be/o6EI-TFBSVA)
