# noir-rover — Ideas Backlog

Ideas brainstormed in session 2026-05-08. Ranked by cool-to-effort ratio.
The Knowledge Graph (idea #1 evolved) + Daily Diary (#5) are being built first.

---

## ✅ In progress — Knowledge Graph + Voice Recall + Daily Diary
- Persistent Kuzu graph DB of notable objects/events/people
- Voice query "have you seen my mug?" → spoken answer + image flash
- Daily diary: NOIR summarises the day's new KG entries in his voice
- Dashboard KNOWLEDGE tab: vis-network graph view + diary pane

---

## 🗂 Backlog

### 2 — NOIR Detective Mode
**Pitch.** Toggle "detective mode" → robot drives a slow patrol, periodically
captures a frame via VLM, but the agent rewrites every observation in
hard-boiled noir voice and speaks it via TTS: *"Coffee mug, half empty. The
kind of mug a man leaves when he doesn't expect to come back."* All
observations are appended to a "case file" (Redis list) readable from the
dashboard.

**Built on.** Patrol primitives (timed move_forward + turn_*), VLM, TTS,
agent persona (already dry/clipped — just add a mode-specific system-prompt
addendum).

**New work.** A `mode:detective` Redis flag + a small loop in bridge-api.py
that drives, captures, asks VLM "describe this in 1980s detective-novel voice",
speaks it. Append to `case_file:today`. ~200 lines.

**Difficulty: M** (1 weekend).

---

### 3 — Sentry mode (patrol when nobody's home)
**Pitch.** When face_rec has seen no enrolled faces for N minutes, robot
enters sentry mode: slow patrol, VLM "is anything notably different from
baseline?" On a hit → posts to a phone webhook (Pushover / ntfy / Discord)
with thumbnail + caption. On enrolled-face return → exits sentry, says
*"welcome back, Akash"*, recaps: *"3 hours of nothing. The cat moved twice."*

**Built on.** Face recognition + KG memory stream + VLM + patrol services.

**New work.** A "baseline" describer, a mode loop, a webhook poster
(env-configurable URL). ~300 lines.

**Difficulty: M** (1 weekend).

---

### 4 — Hide and seek
**Pitch.** *"Let's play hide and seek."* Robot drives to a wall, counts to 30
via TTS. You hide. Then it drives a search pattern — look_around + face_rec at
each heading — until it spots an enrolled face, follows briefly, declares
*"found you."* Timeout: *"I give up."*

**Built on.** Face follow loop + face_rec + STT + TTS + look_around.

**New work.** A small game state machine. ~250 lines.

**Difficulty: M** (1 weekend).

---

### 6 — Show-and-tell face enrollment (voice-driven)
**Pitch.** *"This is Sara."* Robot points camera, captures, saves face
embedding under that name — no manual `faces/Name.jpg` step. Voice enrollment
makes face_rec accessible to non-engineers.

**Built on.** STT + face detection + filesystem write to `faces/`. Re-trigger
insightface enrollment.

**New work.** New agent tool `enroll_face(name)`, voice flow in NOIR system
prompt. ~100 lines.

**Difficulty: S** (1 evening).

---

### 7 — Curious explorer (robot asks YOU questions)
**Pitch.** Toggle "curious mode" → every minute, robot picks a salient object
via VLM, drives toward it, asks the user via TTS *"what's this thing on your
desk?"*, listens via STT, stores `{ts, object, user_answer}` to a "things I
learned today" log. Inverts the usual robot-as-tool dynamic.

**Built on.** VLM + STT + TTS + drive primitives + optional web search.

**New work.** Mode loop, salience prompt, learned-facts list. ~250 lines.

**Difficulty: M** (1 weekend).

---

*Ideas are sized roughly: S = 1 evening, M = 1 weekend, L = 1-2 weeks.*
