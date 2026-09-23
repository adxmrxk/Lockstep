# Running Lockstep on an Elegoo Smart Robot Car V4

The car becomes the body. Lockstep and your control code run on a computer.

```
[Elegoo car]                          [your computer]
  ESP32  :100  <---- WiFi / TCP ---->  elegoo_bridge
  UNO R3 (motors, ultrasonic)                |
                                    robot/range  robot/drive
                                             |     ^
                                          avoider (your control code)
```

**Nothing runs on the Arduino or the ESP32.** They keep their stock firmware.
Lockstep needs an operating system with processes and virtual memory — that is
the entire reason `offset_ptr` exists — and a 2 KB ATmega has none of those. The
microcontrollers stay as they are and speak their normal protocol over WiFi.

---

## 1. Try it with no robot at all

Do this first. It proves your build works before any hardware is involved.

```sh
# terminal 1 -- a fake robot with synthetic sensors
./robot/elegoo/elegoo_bridge --sim --duration 10000

# terminal 2 -- your control node
./robot/elegoo/avoider --duration 8000 --journal /tmp/drive.jrnl
```

You should see the avoider making decisions and the bridge forwarding them.
Then replay the session:

```sh
./robot/elegoo/avoider --replay /tmp/drive.jrnl
./robot/elegoo/avoider --replay /tmp/drive.jrnl
```

Both print the same output hash. That is the whole point of the project, and it
works before the car is out of its box.

---

## 2. Connect the real car

**Power on the car.** The ESP32 module starts its own WiFi access point.

**Join that network** from your computer. It is usually called something like
`ELEGOO-XXXXXX`. Once joined, the car is at **192.168.4.1**.

**Check you can reach it:**

```sh
ping 192.168.4.1
```

**Verify the protocol against YOUR firmware.** This is the important step:

```sh
./robot/elegoo/elegoo_bridge --probe
```

It sends each command and prints the raw reply:

```
  COMMAND            SENT                               RAW REPLY
  heartbeat          {"N":110}                          {ok}
  ultrasonic         {"N":21,"D1":2}                    {47}
  move fwd (slow)    {"N":3,"D1":3,"D2":100}            {ok}
```

- **Sensible replies and the car twitches on the move command** → you are done,
  the protocol matches.
- **Empty replies or nonsense** → your firmware uses different command numbers.
  Edit the `profile` struct at the top of [protocol.hpp](protocol.hpp). That is
  the only place that needs changing; the bridge, the tests and the control node
  are all written against it.

This step exists because I wrote `protocol.hpp` from Elegoo's published protocol
without a car to check it against. Everything else in this directory is tested;
that one file is the only part your hardware has to confirm.

**Then run it for real** — put the car on the floor with space around it:

```sh
./robot/elegoo/elegoo_bridge --host 192.168.4.1 --duration 30000
./robot/elegoo/avoider --duration 28000 --journal ~/drive1.jrnl
```

---

## 3. The part worth showing someone

Drive the car around. Let it do something you did not expect — bump a chair leg,
spin in place, freeze for a moment. Then, at your desk, with the car switched
off:

```sh
./robot/elegoo/avoider --replay ~/drive1.jrnl
```

Every decision it made, in order, from the same sensor readings at the same
timestamps. Run it a hundred times and get the same answer a hundred times.

That is the thing a ROS 2 bag cannot do. A bag gives you the data back; it does
not give you the *timing*, so the run diverges and the bug you were chasing does
not reappear. Here the clock reads are journalled too, so a decision that
depended on *when* something arrived reproduces exactly.

The `decide()` function in [avoider.cpp](avoider.cpp) deliberately branches on
the clock — which way it turns depends on the millisecond — precisely so this
demonstration is not vacuous. If replay did not reproduce the clock, the turns
would differ and the hashes would not match.

---

## Safety

The car stops in two independent ways, because a runaway robot is worse than a
stopped one:

1. `elegoo_bridge` sends an explicit stop on every exit path, Ctrl-C included.
2. The stock firmware halts the motors if it hears nothing for roughly half a
   second — so if the bridge is killed outright and never gets to run (1), the
   car still stops on its own.

The heartbeat exists only to keep (2) quiet during normal operation.

First run on the floor, not a table.

---

## Options

**`elegoo_bridge`**

| Flag | Meaning |
|---|---|
| `--host <ip>` | robot address, default `192.168.4.1` |
| `--port <n>` | robot TCP port, default `100` |
| `--bus <name>` | segment name, default `lockstep-robot` |
| `--attach` | join an existing bus instead of creating one |
| `--rate <hz>` | sensor poll rate, default 20 |
| `--duration <ms>` | run time; omit for until-Ctrl-C |
| `--sim` | no robot: synthetic sensors |
| `--probe` | send each command once, print replies, exit |

**`avoider`**

| Flag | Meaning |
|---|---|
| `--bus <name>` | segment to attach to |
| `--journal <path>` | record the session |
| `--replay <path>` | replay a recording and print its hash |
| `--duration <ms>` | run time |

---

## What is tested, and what is not

Tested, with no robot present (`test_elegoo`, part of `ctest`):

- every command encoder, pinned to an exact string
- reverse encoded as a direction field, because getting that backwards drives
  the car the wrong way
- the motor deadband, so a small speed request becomes a real stop instead of a
  command the hardware silently ignores
- implausible ultrasonic readings rejected, because the HC-SR04 misreads
  constantly and a controller steering on those will swerve at nothing
- the socket layer against a fake robot on loopback: connect, framing, reply
  parsing, fast failure when nothing is listening, and detection of a car that
  is switched off mid-session
- the message types being padding-free, which is what lets a replay hash over
  the whole slot reproduce

**Not tested:** whether the real Elegoo firmware agrees with `protocol.hpp`.
That needs the car, and `--probe` answers it in a minute.
