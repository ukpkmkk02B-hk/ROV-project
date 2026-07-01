# ROV Surface Console Design

## Source

This design is adapted from the local open-design checkout:

```text
external/open-design/plugins/_official/design-systems/mission-control/DESIGN.md
```

The local checkout is intentionally ignored by git through `external/`.

## Product Intent

The Web surface console is an engineering control panel for ROV visual tracking, cooperative docking, manual trim, and runtime parameter tuning. It must feel like an operations console rather than a marketing page.

Primary users are operators and developers who need to:

- See connection, task, motion, and pre-dock status at a glance.
- Send explicit runtime commands without changing command payloads.
- Separate Tracking, Docking, Manual Motion, Motion Safety, and Vision Parameters.
- Read Chinese labels first while keeping English technical terms visible.

## Visual Language

Use a dense mission-control style:

- Dark navy background.
- Compact framed panels.
- Monospace telemetry values.
- Amber for important telemetry.
- Cyan for active data and focus.
- Green for healthy state.
- Orange for warning/disconnected state.
- Red for dangerous actions and STOP.

Avoid decorative illustrations, large hero layouts, cards inside cards, rounded pill-heavy styling, or marketing-style composition.

## Tokens

Core colors:

```text
background:      #0B1120
surface:         #111827
surface-hover:   #1A2535
border:          #1E3A5F
border-subtle:   #162035
text-primary:    #E8F0FE
text-secondary:  #8BA3C7
text-tertiary:   #4A6080
data-primary:    #FFB800
data-accent:     #00D4FF
success:         #26DE81
warning:         #FF9F43
danger:          #FF4757
```

Type:

- UI labels and body: system sans with Chinese fallback.
- Telemetry, status, command values: monospace stack.
- No negative letter spacing.

Shape:

- Panel and button radius: 4px.
- Grid rhythm: 4px baseline, with 8px or 12px gaps.
- No rounded cards above 8px.

## Layout

Top:

- Brand and connection state.
- Manual refresh action.

Status summary:

- Four telemetry cards: System, Task, Motion, Pre-dock.
- Each card has a left accent bar and monospace value.

Main workspace:

- Left column: connection, task diagnostics, motion safety.
- Right column: tracking, docking, manual motion, vision parameters.
- On small screens, columns stack vertically.

## Bilingual Copy

Use Chinese first, English second:

```text
视觉 PID
Visual PID

协同对接 Docking
运动安全 Motion Safety
```

Raw enum values may remain visible when they are useful for debugging.

## Safety Rules

The UI must not change backend command semantics:

- Keep all `data-rov` values unchanged.
- Keep `/api/status`, `/api/connect`, `/api/disconnect`, `/api/command`, and `/api/config` unchanged.
- Do not enable motion by default.
- Keep dangerous actions visually distinct.
- STOP must remain red and immediately visible.

## Reference Files

- `design/reference.html`: static visual prototype, no backend calls.
- `design/tokens.css`: project-local design tokens.
- `tools/surface_console/static/`: runtime implementation.
