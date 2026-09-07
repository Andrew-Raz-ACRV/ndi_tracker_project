# ndi_tracker_project

Electromagnetic tracking capture and processing for the NDI Aurora: records six-degree-of-freedom pose from four sensors to CSV, with MATLAB tooling to turn that into vectors and perform point-based registration.

Built to validate the kinematics of a continuum surgical manipulator against measured tip pose — the ground-truth half of an experiment, rather than the robot half.

**Status:** archived. Research tooling from the PhD, not actively maintained.

## What it does

**Capture.** `CombinedAPISampleDlg.cpp` is NDI's Combined API sample application, modified to log four sensors continuously to a `.csv` file.

**Processing.** MATLAB scripts convert the log into pose vectors and run registration against known points.

## The CSV format

One row per sample. Positions are in **millimetres**; orientations are **quaternions**. Four sensors, labelled `a` to `d`, each contributing seven columns:

```
Sample, Hour, Minutes, Seconds, ms,
Xa, Ya, Za, qoa, qxa, qya, qza,
Xb, Yb, Zb, qob, qxb, qyb, qzb,
Xc, Yc, Zc, qoc, qxc, qyc, qzc,
Xd, Yd, Zd, qod, qxd, qyd, qzd
```

Quaternion order is **w, x, y, z** — `qo` is the scalar term.

There is an example `.csv` in the repository showing real captured data.

## Using it

### Capture

Build the modified sample against NDI's Combined API. **To find what was changed, search the source for `Andrew`** — every modification is marked.

### Processing

```matlab
NDI_csv_to_vectors.m
```

Converts a captured `.csv` into pose vectors. The repository also contains worked examples of point-based registration using known sensor positions.

<!-- RESOLVED 2026-09-07: it is csv. The old README said "csr"; Andrew has corrected
     the actual file to NDI_csv_to_vectors.m, so this draft matches the repo. -->

## Hardware

NDI Aurora electromagnetic tracking system, four 6-DOF sensors.

Tracking accuracy depends on the field generator, the sensor type and the volume you work in — consult NDI's specification for your configuration rather than assuming a figure.

<!-- Deliberate. Andrew, 2026-09-07: "I'm not sure what the accuracy of the NDI Aurora
     is to be honest." That is the right answer to give, and the right thing to do
     with it is state nothing.

     This is validation tooling, so an accuracy number is exactly what a reader wants
     and exactly the most damaging thing to get wrong — a reader who trusts a figure
     here may report it as their measurement uncertainty. Aurora accuracy also varies
     genuinely across field generator, sensor and working volume, so there is no
     single correct number to look up even in principle.

     Pointing at NDI's spec is more useful than a half-remembered figure and cannot
     be wrong. If Andrew ever quoted a figure in the thesis or the T-Mech paper for
     his specific setup, THAT would be worth adding here with the source named. -->


QUT students: there is an [instruction guide](QUT%20Northern%20Digital%20Inc%20Aurora%20Instructions.pdf) for the lab system in this repository.

## Related

The manipulator whose kinematics this was used to validate: [SnakeRaven-Project](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project).

## Licence

**MIT** — see [LICENSE](LICENSE) — for the MATLAB tooling, the data-processing scripts and the modifications in this repository.

**`CombinedAPISampleDlg.cpp` is not covered by that licence.** It derives from Northern Digital Inc.'s Combined API sample code, which carries its own copyright and is provided by NDI "as is" without warranty of any kind. NDI's notice is retained at the top of the file and governs it. If you intend to reuse or redistribute that file, obtain the sample and its terms from NDI directly.

> Portions Copyright © 2002, 2003 Northern Digital Inc. All rights reserved.

See [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).

<!-- RESOLVED 2026-09-07. Andrew supplied the header from the file itself. Reading it:

     - It is a WARRANTY DISCLAIMER and liability limitation, not a licence grant. It
       says the sample "is licensed and provided as is" without ever stating the terms
       of that licence — those live in the CAPI SDK's own agreement, not in the file.
     - It ends "Copyright (C) 2002, 2003, Northern Digital Inc. All rights reserved."

     "All rights reserved" with no redistribution grant means MIT definitively cannot
     cover this file. That is now stated explicitly above rather than left implied by
     a blanket repo licence.

     WHAT ANDREW HAS ALREADY DONE RIGHT: the NDI notice is intact at the top of the
     file. That is the thing that would have been hard to fix later and it is done.

     THREE LEVELS OF RESPONSE, and the draft implements the first:
     1. Carve it out in the README and LICENSE, keep NDI's header intact. Low effort,
        honest, and what most research repos in this position do. Implemented above.
     2. Check the CAPI SDK's licence agreement for its actual redistribution terms —
        that is where the grant lives, and it may well permit exactly this. Worth
        doing if the repo ever matters commercially.
     3. Belt and braces: ship only a patch against NDI's sample plus instructions to
        obtain it, rather than the whole file. More friction for every reader, and
        only warranted if (2) turns out to prohibit redistribution.

     Recommend stopping at 1 unless something changes. Also add the same carve-out
     to the LICENSE file itself, not just the README — that is the file people check.

     NOT LEGAL ADVICE, and not a reason for alarm: this is an ordinary situation for
     research code built on a vendor SDK, and the fix is a paragraph. -->


## Questions

Written by Andrew Razjigaev. Questions: andrew_razjigaev@outlook.com
