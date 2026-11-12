---
name: interactive-robotics-sponsor-showcase
description: >-
  End-to-end runbook for designing, porting, verifying, and deploying interactive 2D
  robotics or technical showcases for sponsors and non-technical stakeholders. Use this
  skill whenever asked to build an interactive web demo, port robot control/navigation
  code (e.g. FTC/Java/holonomic/spline) to a 2D visualization, pitch a technical system
  to sponsors, or deploy a static showcase to GitHub Pages without UI clutter or silent bugs.
---

# Interactive Robotics & Technical Sponsor Showcase Skill

This skill encodes the exact architecture, UX principles, verification gates, and deployment runbook required to turn complex robotics/control algorithms into a **self-explanatory, sponsor-ready interactive 2D web showcase** on the first attempt—without engineering slop, broken scripts, or false assumptions.

---

## 1. Core Mindset: Sponsor Pitch vs. Engineering Debug Tool

Before writing a single line of HTML or Canvas code, apply these mandatory product rules:

1. **Translate Competition Jargon into a Real-World Use Case**:
   - **NEVER** expose raw competition specifics (e.g., FTC field triangles, season-specific scoring zones, internal variable names like `smallTriangle`, `loadSpike`, `goToK`, or raw wheel power bars) in the main UI.
   - **ALWAYS** frame the technology around an immediately understandable, high-value real-world scenario (e.g., an **Autonomous Smart Office / Warehouse Courier Robot** that undocks, navigates to a library bookshelf, picks up 3 books one by one into a visible cargo basket, routes around room obstacles, delivers them to a client's executive desk inbox, and loops continuously through new batches).
2. **Choose 2D-Legible Mechanisms Only**:
   - In a 2D top-down view, **do not** simulate vertical/3D projectile launchers that look invisible from above.
   - Instead, use mechanisms whose kinematics are 100% obvious from top-down:
     - **Telescopic front arm + dual gripper jaws** that physically extend into a pickup zone and retract with the item.
     - **Top-mounted 3-slot cargo basket** where collected items are drawn in real time (and mirrored in the UI sidebar).
     - **Mecanum wheels with 45° rollers** and an **LED status halo** around the chassis indicating current operating mode (Autonomous vs. Human Pilot Override vs. Docking).
3. **Radical UI Simplicity (Zero Clutter)**:
   - **No checkbox clutter**: Do NOT put checkboxes for "Show Path", "Show Trail", "Show Room", or "Continuous Loop". Hardcode all of them to **`true`** so the presentation always looks rich and complete.
   - **No redundant manual-stage buttons**: Do NOT clutter the toolbar with buttons for individual sub-steps ("Go to Shelf", "Go to Desk", "Extend Slide"). Keep the toolbar to **at most 3 buttons**:
     1. `▶ Pornește / ⏸ Pauză Misiune` (Primary CTA)
     2. `👋 Simulează Ocolire (Pilot Nudge)` (Demonstrates live human + autonomous shared control)
     3. `↺ Reset`
   - **Auto-Start on Load**: Automatically trigger `startMission()` ~600ms after page load so the sponsor immediately sees the robot moving smoothly without having to hunt for a start button.
   - **Tuck Engineering Telemetry Away**: Place mathematical readouts (X/Y, heading, velocity, spline segment `i / localT`, battery compensation) inside a collapsed `<details>` accordion at the bottom of the right panel labeled *"Parametri Tehnici & Cinematică (Pentru Ingineri)"*.

---

## 2. Visual & Spatial Design Checklist (2D Canvas)

1. **Generous Landmark Proportions**:
   - On a `760×580` canvas (`SCALE ≈ 2.15 px/cm`), make architectural landmarks large, unmistakable, and detailed:
     - **Bookshelf / Storage Rack** (`~94×240 px` on the west wall): multi-tier hardwood frame, colorful book volumes with readable titles (`Vol.1 ...`), and a dashed amber `ZONĂ PRELUARE` staging pad.
     - **Executive Desk & Client** (`~96×170 px` on the east side): mahogany surface, dual monitors, desk lamp with radial glow, green `📥 INBOX` tray where delivered items stack up, and a seated client with dynamic speech bubbles (*"Aștept 3 cărți..."* ➔ *"🎉 3 Cărți Primite! Mulțumesc!"*).
     - **Obstacles** (central lounge table with plant, armchair) positioned so the spline visibly curves around them with `>30 cm` clearance.
2. **Illuminated Trajectory & Waypoint Nodes**:
   - Render the Catmull-Rom spline as a smooth dashed cyan curve.
   - Extract unique waypoints along the path and render each node with:
     - Outer glowing halo + stroke ring (cyan for `START` / `P2` / `P3`, emerald green for `DEST`).
     - Crisp white inner core dot.
     - **Directional heading tick** pointing along `wp.heading` to visually prove independent holonomic rotation.
     - Compact pill badge (`START`, `P2`, `P3`, `DEST`).

---

## 3. Porting Control & Physics Without Bugs

1. **Coordinate System Fidelity**:
   - Check the source codebase's angle convention before writing transforms. For example, in Roary (`Point.java`), `atan()` is `atan2(x, y)` (`0° = +Y` up, `90° = +X` right, `-90° = -X` left). Document this at the top of `sim.js`.
2. **Autonomous Multiplier Activation (`goToK`)**:
   - When motor power is composed as `motor = goTo * goToK + mainDrivePowers`, ensure `drive.setGoToK(true)` and `drive.DXEnabled = true` are explicitly called inside `startMission()`. If `goToK` is left at `0`, autonomous spline powers are multiplied by zero and the robot sits still.
3. **Zero Creep on Stop & Smooth Inertia**:
   - Filter wheel commands through realistic AGV mass/inertia (`accelRate`) and zero out autonomous powers when `CURRENT_SPLINE` is empty or the robot is docked in pickup/delivery states.

---

## 4. Mandatory Pre-Response Verification Gates

**NEVER** claim a web showcase works based only on Node unit tests. Always run all three verification gates before replying to the user:

### Gate 1: Shared Global Scope & DOM ID Audit
Plain browser `<script src="...">` tags share a **single global lexical scope**. Declaring `const clamp = ...` in two separate script files passes in CommonJS `require()` tests but throws `Uncaught SyntaxError: Identifier 'clamp' has already been declared` in the browser, killing the entire second script!
- Run the bundled verification script:
  ```bash
  node .agents/skills/interactive-robotics-sponsor-showcase/scripts/verify_web_bundle.js web/index.html
  ```
  This script executes all `<script>` tags from `index.html` inside a single shared `vm.Context` and verifies that every `document.getElementById('...')` call in JS matches an actual `id="..."` in `index.html`.

### Gate 2: Headless 60 Hz Full-Cycle Simulation Test
- Run a headless multi-batch simulation (~2,500 ticks at `DT = 1/60`) asserting:
  1. Robot undocks and reaches the shelf (`dist < 6 cm`).
  2. Telescopic arm picks up all 3 items (`basketCargo.length === 3`).
  3. Pilot nudge (`joyX`, `joyY`) displaces the robot and Roary smoothly converges back onto the spline without resetting.
  4. Robot docks at the desk, delivers all 3 items (`totalDelivered === 3`), and automatically starts Batch 2 (`totalDelivered === 6`).

### Gate 3: Real Headless Browser Render & Screenshot Inspection
- Render the actual `file://.../web/index.html` in headless Brave/Chrome with virtual time advanced past the 600ms auto-start:
  ```bash
  "/Applications/Brave Browser.app/Contents/MacOS/Brave Browser" \
    --headless --virtual-time-budget=2500 \
    --screenshot="/tmp/showcase_verify.png" \
    --window-size=1400,900 "file://$PWD/web/index.html"
  ```
- Copy `/tmp/showcase_verify.png` to the conversation artifact directory and inspect it with `read_file` to visually confirm the canvas is not black, the robot is moving along the spline, waypoints are labeled, and no UI elements overlap.

---

## 5. GitHub & GitHub Pages Deployment Runbook (macOS / iCloud Safe)

1. **Never Touch Existing User Repositories**:
   - Only create or push to a dedicated repository explicitly named for the showcase (e.g., `roary-presentation`).
2. **Pre-Flight Check for macOS iCloud `dataless` Git Objects**:
   - Workspaces inside `~/Documents` on macOS are often synced with iCloud Drive, which can evict files in `.git/objects/` into `dataless` stubs, causing `git push` to fail with `fatal: mmap failed: Operation canceled`.
   - Before running `git push`, always check and materialize any evicted objects:
     ```bash
     ls -lO .git/objects/*/* 2>/dev/null | awk '/dataless/ {print $NF}' | xargs -n 1 brctl download
     ```
3. **Dual-Mode GitHub Pages Configuration**:
   - Create both:
     - `.github/workflows/deploy-pages.yml` using `actions/upload-pages-artifact@v3` (`path: 'web'`) and `actions/deploy-pages@v4`.
     - A root `index.html` that redirects to `web/` (`<meta http-equiv="refresh" content="0; url=web/" />`) so Pages works regardless of whether the repo uses *GitHub Actions* or *Deploy from a branch (`/`)*!
4. **Verify Live HTTP 200**:
   - After pushing, watch the workflow run (`gh run watch`) and verify `curl -sI https://<user>.github.io/<repo>/` returns `HTTP/2 200`.
