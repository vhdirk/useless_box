# Useless rust box

Launch openocd:

```sh
openocd
```

You should see terminal output like this:

```sh
Open On-Chip Debugger 0.10.0
[...]
Info : stm32f1x.cpu: hardware has 6 breakpoints, 4 watchpoints
```

Open a new terminal, compile and flash

```sh
cargo run
```

Now, the program is flashed, and you are on a gdb prompt. Type c (for continue) you can see the on board LED blinking.