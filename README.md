# rs-esp32c3-slint-st7789-demo



- this is a simple demo of slint ui, using rust and esp32c3, in no-std env.
    ref from  rs-esp32s3-slint-st7789-demo

- prepare tools
	install espflash
    ~~~shell
	cargo install espflash
    ~~~

- build

    ~~~shell
    cargo build --release
    ~~~
    
- flash
    ~~~shell
    windows:
        espflash.exe flash .\target\riscv32imc-unknown-none-elf\release\rs-esp32c3-no-std-st7789-demo --monitor

    Linux:
        espflash flash ./target/riscv32imc-unknown-none-elf/release/rs-esp32c3-no-std-st7789-demo --monitor
    ~~~