# USART + DMA на stm32f407vg

Цель программы:

В USART2 записываются данные, и передаются в USART1. все это происходит в DMA.

## компиляция/прошивка на stm32f407vg discovery

Скомпилировать

```
make all
```

Прошить stm32f407vg, но в целом такой командой и другие f4 прошиваются

```
$ sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c \
"init; reset halt; flash write_image erase main.hex; "\  
"reset; exit"
```


