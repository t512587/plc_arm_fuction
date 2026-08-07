# Y1 / Y2 slot state and vacuum guard

The current machine configuration has vacuum command points but no configured
cargo-presence or vacuum-pressure feedback. Slot occupancy is therefore an
operator-confirmed state, not an automatically detected state.

## Occupancy

- `unknown`: cargo presence is not confirmed. If the slot was previously
  occupied, vacuum protection remains active.
- `empty`: cargo has been confirmed absent. Selecting this requires an explicit
  release confirmation and turns that side's vacuum off.
- `occupied`: cargo is present. The service turns break-vacuum off, turns vacuum
  on, reads the command back, then persists the state.

## Safety invariant

While a slot has `vacuum_required: true`, all point-based and raw bit write
paths reject:

- vacuum `OFF` on the occupied side;
- break-vacuum `ON` on the occupied side.

Only the explicit `empty` confirmation path may bypass the guard. A watchdog
checks once per second after PLC connection and restores vacuum if an occupied
side reads back as off.

## API

```text
GET  /slots
PUT  /slots/Y1
PUT  /slots/Y2
POST /slots/enforce
```

Example occupied confirmation:

```json
{
  "occupancy": "occupied",
  "cargo_id": "BOX-001",
  "purpose": "target_to_pick",
  "shelf_id": "A01",
  "shelf_level": 2,
  "confirm_release": false
}
```

Example empty confirmation:

```json
{
  "occupancy": "empty",
  "purpose": "unknown",
  "confirm_release": true
}
```

State is stored at runtime in `plc2/runtime/slot_states.json`. The file is not
created until the first successful operator update.
