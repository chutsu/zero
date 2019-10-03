# zero

Just bits and bobs.


## STM32

  Note: On an STM32F103C8 chip, the flash memory starts from 0x80000000 (see
  chapter 4. "Memory Mapping" in the datasheet).

  Read flash memory starting from 0x80000000 saving 4K worth of data

    st-flash read firmware.img 0x80000000 0x1000

  Write flash memory starting from 0x80000000

    st-flash write firmware.img 0x80000000

  Erase

    st-flash erase

## LICENCE

GPLv3
