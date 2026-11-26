# The OCRAM mailbox (proposed)

A proposed third path between the HPS and a core, beside the GPO/GPI ("SPI") link and the LW bridge window of `docs/LW_BRIDGE.md`. It is for bulk: OSD rows, disk blocks, replies. The HPS writes records into its own 64 KB **on-chip RAM** (OCRAM, physical `0xFFFF0000`), and the core fetches them as an AXI master on the Cyclone V **FPGA-to-HPS bridge**, whose atom `sys/` already instantiates with no port connected.

**Status: the floor, the OSD tier, the SECTOR tier and the download tier are built, for test cores only.** `MISTER_OCRAM` builds the bridge master, the run latch, the poller, the hello, the EVT ring's writer and its line, and the consumer handshake. Nothing else in this document is built: no PAD ring, and no H_BIND LANE or PAD function. The SECTOR, download, mount, UIO, events and capture tiers are benched, not yet fitted or run on hardware; several facts the floor rests on are unverified ("What is not verified", at the end). It is published with the LW framework so that core and host authors can read both paths side by side and say which they would build.

## The menu

Every path is its own macro, off by default; a path that is not defined builds no logic. An OCRAM tier needs the OCRAM floor, as an LW tier needs `MISTER_LW`. Naming two paths for one function builds both, and the host binds one. The two floors share the crossing cells of `sys/lw_cdc.sv` and the HPS reset that `sysmem` exports, which `MISTER_LW` alone builds today; the floor's change widens both to either floor.

| macro (proposed) | builds | needs | cost (ESTIMATED, by analogy with the fitted LW blocks less their FIFOs) |
|---|---|---|---|
| `MISTER_OCRAM` | the floor: the bridge master, its arbiter and poller, the hello, the core-to-host ring's writer and its interrupt line, one GPI bit | - | 180-260 ALMs, no block memory |
| `MISTER_OCRAM_OSD` | OSD rows from the OSD ring into each `osd`'s `osd_buffer`, through the LW OSDBUF writer's port | the floor (`ocram_tier_needs_MISTER_OCRAM`) | 60-80 ALMs |
| `MISTER_OCRAM_PAD` | pad stores from a ring | the floor | 40-60 ALMs |
| `MISTER_OCRAM_BELL` | an LW register whose store starts a poll at once (only if the poll proves too slow) | both floors | 10-20 ALMs |
| `MISTER_OCRAM_POLL` | the idle poll period, as a power of two of `clk_100m` cycles (default 12: 41 us) | the floor | - |

**The set menu.** Move bulk to the mailbox and pads to the LW window; leave status and events on the link unless a measurement says otherwise, because each of them costs more fabric off the link than on it. Every cost is ESTIMATED unless marked fitted.

| set | macros | status, events, saves | cost beyond stock | for |
|---|---|---|---|---|
| add events on LW | the default plus `MISTER_LW_EVT` | status and flags on an LW line; saves on SPI | about +120 ALMs (104 fitted) | a core whose status or block-request latency is measured to matter, and that has the room |
| LW only | the tiers of `docs/LW_BRIDGE.md`, save slots included | LW | about 1,065 ALMs + 5 M10K (fitted, a disc core's build) | a cartridge core; a core that wants save blocks off SPI |
| lean, no LW bridge | `MISTER_OCRAM`, `MISTER_OCRAM_OSD`; pads on SPI | SPI | about 160 + 160 ALMs, 0 M10K (ESTIMATED) | a core that wants no bridge the HPS must be answered on. A disc core cannot take it yet: the SECTOR tier needs `MISTER_LW` |
| area-critical extras | `MISTER_DISABLE_SPI_PAD` (needs `MISTER_LW_PAD`); `CONF_STR_BRAM(1)` | — | about -85 ALMs (ESTIMATED; a stock Main loses the pads); about -350 ALMs for 4 M10K (ESTIMATED) | only when the fit needs it; never a default |

## How it works

### The master

One module wires the existing `cyclonev_hps_interface_fpga2hps` atom at its 32-bit width and is the only logic that drives it.

- **It runs on `clk_100m`** (`h2f_user0_clk`), and no fabric reset holds it. It never abandons a transaction it has started: a core reset, a stopped `clk_sys` or a host that has gone cannot leave one in flight.

### The layers

The master and the consumer are each a wrapper over one module a job, and every module with logic but the crossings (`ocram_xing`, `hps_ocram_evt`) has one clock. A boundary between them is ports, never an SV `interface`.

| layer | module | file | clock | holds |
|---|---|---|---|---|
| config | - | `sys/ocram_cfg.vh` | - | the `.qsf` macros as localparams, once: the tiers' enables, the geometry, the select mask, C_CAPS. `sys_top.v`, `hps_ocram_master.sv` and `hps_io.sv` include it |
| bus | `ocram_axi` | `sys/ocram_axi.sv` | `clk_100m` | the AXI3 engine, one single-beat transaction at a time |
| access | `ocram_arb` | `sys/ocram_arb.sv` | `clk_100m` | the pick over the request ports, fixed order then the consumers round robin; a mux of held requests |
| session | `hps_ocram_master` | `sys/hps_ocram_master.sv` | `clk_100m` | the run latch, the phase, the poller, the hello's stores, C_STATE, C_BIND, the two lines; the record sources |
| records out | `ocram_evt_wr` | `sys/ocram_evt_wr.sv` | `clk_100m` | the EVT ring's head, tail and free words; one record at a time from its sources, and its abandon |
| records in | `ocram_ring_rd` | `sys/ocram_ring_rd.sv` | `clk_100m` | the walker: the ring table by ring id, the head read, the length check, the fetch, the tail store, the faults; one `ocram_bind` (`sys/ocram_bind.sv`) a block slot |
| mount write | `ocram_mnt_wr` | `sys/ocram_mnt_wr.sv` | `clk_100m` | a mount WRITE's words, pulled from the feeder through the crossing for its record |
| crossing | `ocram_xing` | `sys/ocram_xing.sv` | both | every `lw_xfer`, `lw_sync` and `lw_fence` between the walker and the sinks |
| sinks | `ocram_osd_wr`, `hps_ocram_sfeed`, `hps_ocram_fplay`, `ocram_blk_req` | `sys/ocram_osd_wr.sv`, `sys/hps_ocram_rx.sv` (both), `sys/ocram_blk_req.sv` | `clk_sys` | one consumer's rules each: the OSD writer, the SECTOR feeder, the download player, the block slots' request records |
| link words | `lw_link_seq` | `sys/lw_lane.sv` | `clk_sys` | a player's link-word sequencer (own, hold, the select, the strobe, the gap), shared by the download player and the LW lane's; each player keeps its source and answers |

### The map

Offsets are from `0xFFFF0000`. The fabric declares this geometry in the control page and a host reads it there, so a later layout needs no host change.

| offset | size | ring | *r* | direction | carries |
|---|---|---|---|---|---|
| `0x0000` | `0x0100` | control page | - | both | the hello, binds, every index, the fabric's descriptors |
| `0x0400` | `0x0400` | PAD | 0 | host to core | pad stores (not built; never in a build with the mount ring) |
| `0x1000` | `0x1000` | LANE | 1 | host to core | link frames |
| `0x2000` | `0x2000` | OSD | 2 | host to core | image rows, 65 words each |
| `0x4000` | `0x2000` | EVT | 3 | core to host | status, block requests, writes, binds, replies |
| `0x7000` | `0x8000` | SECTOR | 4 | host to core | block read frames: thirteen of 2,352 bytes |
| `0xF000` | `0x1000` | never touched | - | - | the boot ROM's workspace |

*r* is the ring's fixed number in every per-ring word of the control page. A ring whose tier is not built has no descriptor, and its span is never read or written.

### Rings and records

- **A ring** is a power-of-two count of 32-bit words with two indices, each a free-running word count modulo 65,536 and each a full word in the control page. **Each index is written by one side only:** the producer's head, the consumer's tail. `(head - tail) & 0xFFFF` words are waiting, and the ring is full when that equals its size.
- **A record** is one header word, `{kind[31:24], arg[23:16], len[15:0]}`, then `len` payload words. It may wrap the end of its ring.

### The control page

Each word has one writer. In a host of several threads, H_SECTOR and each H_IDX belong to the thread that serves that ring. A host stores the fabric's half only to zero it, and only while the master is stopped ("The rule that replaces the firewall").

| offset | word | written by | content |
|---|---|---|---|
| `0x00` | H_HELLO | host | `{0x4F43, nonce[15:0]}`, the nonce never 0; 0 = no session. Stored last |
| `0x04` | H_BIND | host | the functions the host takes from SPI for the whole session: EVT `[8]`, LANE `[9]`, PAD `[10]`, OSD `[11]` |
| `0x08` | H_LINE | host | bit *r*: ring *r*'s interrupt line may rise |
| `0x10` + 4*r* | H_IDX[*r*] | host | its head of a host-to-core ring, its tail of a core-to-host ring |
| `0x80` | C_HELLO | fabric | the master's state, below; in a session, the echo of H_HELLO, stored last |
| `0x84` | C_VERSION | fabric | `{major, minor}` of the layout |
| `0x8C` | C_STATE | fabric | `[7:0]` hellos taken since configuration; `[12:8]` ring *r* is stopped on a fault; `[20:16]` ring *r* ended the session before this one mid-record |
| `0x90` + 4*r* | C_IDX[*r*] | fabric | its tail of a host-to-core ring, its head of a core-to-host ring |
| `0xB0` + 4*r* | C_RING[*r*] | fabric | `{kind[31:27], line ordinal[26:22], log2 words[21:18], 0[17:14], offset in words[13:0]}`; kind is the function's kind in the LW directory (PAD 1, SECTOR 2, OSD 3, EVT 4, LANE 5); 0 = absent |

| C_HELLO | the master is | stored by |
|---|---|---|
| 0 | unknown: no master has stored since the host's zero | the host, before every rising edge of the run bit |
| `0x4F430000` | running, armed, in no session | the master, on reading H_HELLO = 0 |
| `0x57540000` | running, waiting for H_HELLO = 0 | the master, on reading any other H_HELLO it has not echoed |
| `{0x4F43, nonce}` | in the session of that hello | the master, last of the hello's stores |
| `0x51540000` | stopped | the master, as the last store of a run |

## The host's side

### Mapping

Map `0xFFFF0000`, 64 KB, **uncached and strongly ordered**: on ARMv7 Linux an `mmap` of `/dev/mem` over this range is exactly that, because the range is not kernel RAM. Each store is then one bus write that completes before the next begins; nothing merges, repeats or reorders; and no cache on either side holds a copy. There is nothing to flush and no barrier to issue. A cached mapping is not legal. Nor is a write-combined one as this guide stands: its stores would need an ordering step before each publish that nobody has measured.

### Before the first store: the warm-boot rule

After boot the on-chip RAM still holds the preloader, and a current U-Boot arms the boot ROM to run that image again on a warm reset (the system manager's `romcodegrp.warmramgrp.enable`, `0xFFD080E0`, holds `0xAE9EFEBC`). The mailbox overwrites the image. **So a host stores 0 to that register before its first store to the on-chip RAM**; a warm reset then takes the boot ROM's path through the boot medium, as a cold boot does. Whether that path boots a given board after a warm reset is not verified here. A host that skips the store risks a machine that does not come back from a warm reset; a host that makes it must know how its board behaves. If that path proves unusable, the map moves above the preloader's image and the register is left alone; a project then builds every core and every host for that one map, because nothing a host reads before its first store tells two maps apart.

### Discovery and the hello

Reading the on-chip RAM can never hang, so nothing here needs a firewall. What a host needs is proof that a live master in *this* bitstream is on the other side: the RAM outlives every core load and still holds the last core's words.

### The rule that replaces the firewall

The mirrored danger is **a fabric transaction in flight while the bridge is reset or the FPGA is programmed.**

- **Raise the run bit only while the FPGA-to-HPS bridge is out of reset,** and hold it until the armed read.

### Doorbells

- **Host to core: none.** The master polls the host half every `2^MISTER_OCRAM_POLL` cycles, and back to back only while a record can be consumed or a line is high. The worst case, 41 us by default, is invisible to an OSD row and small beside a block's service time.
- **Core to host: one level line per ring that needs one,** declared in the ring's descriptor with the ordinals of `docs/LW_BRIDGE.md` ("Interrupt lines"). It is high while `C_IDX != H_IDX`, and while it is high the master re-reads the host's tail back to back, so the line falls within a few transactions of the store that empties the ring. A host that re-arms sooner sees one more interrupt and an empty ring; its drain must take one empty pass. A host with no interrupt node reads C_IDX from its loop instead: one memory read.

### Records

| ring | kind | payload | meaning |
|---|---|---|---|
| OSD | `0x01` ROW, `arg` = row | 64 words: the row's 256 bytes | written into `osd_buffer` as the LW OSDBUF writer does, under the same guard |
| SECTOR | `0x10` FRAME, `arg` = `{blk_cnt[5:0], k[1:0]}`, 0 | the LBA, then the frame's words | played into the slot if it answers the core's standing request |
| EVT | `0x21` REQUEST, `arg` = `{blk_cnt[5:0], k[1:0]}` | the LBA | the bound slot's request stands and the ring holds no answer, including a request that already stood when the bind arrived |
| EVT | `0x23` MARK, `arg` = ring | its deepest fill | a ring's high-water mark, for sizing |
| EVT | `0x24` CONFSTR | the string, four bytes a word | the configuration string, once after the hello |
| PAD | `0x40` STORE, `arg` = pad | two words | one pad set-command |

- **OSD.** A byte in rows 8-15 sets highres, as an LW or SPI write does, so a host sends an SPI OSD control word only when the OSD ring is drained (`C_IDX` equals its head), and binds the OSD before its first row. There is no atomic flip: a repaint is visible as it lands.
- **FRAME, REQUEST and WRITE share one `arg`,** `{blk_cnt[5:0], k[1:0]}`. For k = 0 (the SECTOR slot) it is 0 in both directions: the frame's length is C_K[4]'s.

## What a host must not assume

- that the mailbox exists because the RAM holds a plausible control page: only GPI bit 26, the armed value and the echo of its own nonce say so;
- that a record is seen before its head is stored, or that a head may be stored before its record;

## Timing rule

A core that builds an OCRAM tier ships only with a timing report in which no failing setup, hold, recovery or removal path starts or ends in the master or in any `ocram_*` module. A failing path in the master can leave a transaction unfinished, which is the danger the gate exists for.
