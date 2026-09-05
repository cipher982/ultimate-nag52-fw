# Firmware for Ultimate NAG52 722.6 TCU

## Links
* [YT playlist](https://www.youtube.com/watch?v=V27D5PrgMk8&list=PLxrw-4Vt7xtu9d8lCkMCG0_K7oHcsSMtF)
* [Store website](http://ultimate-nag52.net/)
* [Wiki](http://docs.ultimate-nag52.net/)

## DISCLAIMER

I am in no way responsible if your gearbox or car dies as a result of using this firmware!
Although this firmware is being tested actively, there are loads of unknowns which may occur rarely during operation that
are not tested yet.

## IMPORTANT

If you have just built your board, or are updating to firmware after 30/11/22, please look at [this guide](https://youtu.be/ov3pYcKIA70)!
Your TCU will be bricked until you follow it!

## NOTICE ABOUT COPIED CODE (For PRs or change requests)
This project contains code built from the ground up, or code based on black-box style reverse engineering of the behaviour of the standard EGS52 module. Submitions of code pull requests based on unethically sourced documentation or code will be immediatly deleted and closed. 

## Generating the code documentation

1. Install [doxygen](https://www.doxygen.nl/manual/install.html)
2. Clone doxygen-awesome-css submodule with `git submodule update --init --recursive`
3. Run `doxygen`
4. View code documentation in the generated `html` folder

## Persistence safety qualification

Desk changes are not loaded-hardware qualification. Before flashing or vehicle
use, preserve the prior image and complete configuration/calibration/adaptation
readbacks. Never qualify a new guard by writing diagnostics in a moving vehicle.

Local checks (no upload):

```sh
python3 tools/check_nvs_isr_guard.py --list
python3 -m unittest discover -s test -p 'test_*guard*.py'
python3 test/test_persistence_host.py
pio run -e unified
```

Record the actual PlatformIO package/framework version and firmware ELF/hash.
The configured platform is `espressif32@6.12.0`; the reviewed local framework
source is ESP-IDF 5.5.0. `CONFIG_IDF_INIT_VERSION` is generated historical
configuration metadata, not evidence of the framework used for a build.

Build target flash size is **4 MiB**, matching the checked-in partition table,
which ends at `0x3C6000`. The earlier 2 MiB SDK setting contradicted that layout.
This establishes the address space required by the build, not the physical
capacity of an arbitrary controller. Before flashing a full bootloader/partition
image, independently verify the target's flash capacity and preserve its existing
layout, firmware and configuration readbacks.

`TccFlashGuard` owns the full operation with a recursive task mutex. A competing
task fails immediately rather than blocking the gearbox behind an OTA session.
The short internal-RAM ISR gate spans callback output changes **and** alarm
reprogramming; closing it drains admitted work before checked GPTimer stop.
No flash write or task delay is performed under that spinlock. Failed stop
rejects persistence; failed alarm/start leaves output inhibited and returns an
error. Restart keeps the hardware count monotonic and chooses a never-reused
deadline at least 10 ms beyond both the stopped count and previous alarm.
Callbacks require matching identity **and** a captured count at/after that
deadline, rejecting old dispatches and relabeled pending interrupts. The lexical
checker covers direct sinks, not runtime control flow or hardware quiescence.
Host checks exercise ownership, rejection/error cleanup, stale timer epochs,
logger/slave recovery, failed-map exclusion and preservation on failed reloads.

All boot default creation precedes explicit timer startup. After boot, persistence
requires fresh P/N and **both** rear-wheel speeds exactly zero; unknown/stale,
intermediate and FOUR..ONE selectors are rejected. Mode reporting remains
read-only; transient as well as persistent device-mode mutations use that gate.
Passive signal decoding and diagnostic CAN reception continue in slave/logger
modes so their fresh safety gate can authorize return to normal; this does not
enable normal transmission or normal controller actuation in those modes.
Bench diagnostics therefore need positively valid passive/stationary input
simulation; absent CAN is not an implicit authorization. OTA additionally
requires known engine-off, keeps task-affine ownership through verification, and
abandons the session on unsafe/stale inputs or timeout. There is no claim that
software can prevent a physical selector change during an individual flash call.

The I2C expander remains live during OTA to preserve TRRS safety inputs. Its
transfers are synchronous (queued transfers could outlive the stack request and
falsely mark data fresh). In the reviewed SDK/config, `CONFIG_I2C_ISR_IRAM_SAFE`
selects internal driver allocations and an IRAM interrupt; the driver's linker
fragment places ISR helpers in no-flash sections. Application RX buffers must
be internal RAM, checked at expander initialization. No application I2C callback
is registered. Verify those properties in the actual release ELF/config; a
different SDK is not covered by this source review.

Bench gate, on a current-limited stable supply and instrumented harness:

1. Cold boot with absent `DEV_MODE`, core/settings/map defaults. Confirm successful
   persistence before timer start and no reboot. Repeated changed **and unchanged**
   profile/map saves must not grow NVS handle use or strand timer ownership.
2. Scope TCC GPIO/current across nested saves, concurrent Park/diagnostic requests,
   OTA completion/failure/timeout and callback alarm boundaries. Confirm output
   inhibition during ownership and fresh-cycle recovery; inspect both legacy
   LEDC and zener paths. Legacy callback register writes replace potentially
   blocking LEDC driver calls, without changing duty thresholds/calibration.
3. Inject active, stale, unknown, FOUR..ONE and nonzero wheel-speed inputs.
   Rejected requests must leave mode and actuator output unchanged. Exercise
   live I2C/TRRS during flash, including NACK/timeouts; stale input must reject.
4. Query read-only KWP `$21 $2F` after every boot. Record uptime/reset reason and
   UART/CAN evidence. Panic, interrupt-WDT or task-WDT is a failed gate; brownout
   requires supply investigation. No persistent reset counter is added.

Only after this gate: a supported vehicle proof run with old image/readbacks
available, no diagnostic writes, and passive CAN/UART capture. Repeated Park
entries must show no simultaneous `0x218`/`0x338`/`0x418` silence/reset signature
or uptime restart. Any reset stops qualification. OTA rollback is not automatic
in the reviewed configuration; preserve a deliberate recovery/reflash path.


## Additional contributors 

* @chrissivo - ClassicCAN contributor


