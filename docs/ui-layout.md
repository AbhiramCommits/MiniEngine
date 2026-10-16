# UI layout file schema

MiniEngine's UI is described by a JSON file (default `data/ui/main.ui.json`) that the app reads at
startup, and that `tests/test_ui.cpp` parses headlessly. Changing the file changes the widget tree,
the computed rectangles, and the keyboard focus order — no recompilation.

A second example lives at `data/ui/alt.ui.json`; it adds a `debug` panel and is used by the test
that asserts "editing the JSON changes the tree".

## Top level

```json
{
  "viewport": { "width": 1280, "height": 720 },
  "padding": 10,
  "widgets": [ /* root widgets */ ]
}
```

| Key | Type | Meaning |
| --- | --- | --- |
| `viewport.width` / `viewport.height` | number | Reference size. The app recomputes the layout whenever the real window differs. |
| `padding` | number | Inset applied to the root frame before root widgets are anchored. |
| `widgets` | array | Root-level widgets. |

## Widget fields

```json
{
  "id": "pos_x",
  "type": "slider",
  "text": "Position X",
  "binding": "selected.position.x",
  "min": -50, "max": 50,
  "anchor": "top-left",
  "x": 0, "y": 0,
  "width": 284, "height": 22,
  "padding": 0, "spacing": 0,
  "direction": "column",
  "focusable": true,
  "children": []
}
```

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `id` | string | — | Unique name. Used as the ImGui window id, the focus id, and for `Layout::find`. |
| `type` | string | `"panel"` | `panel`, `label`, `button`, `slider`, `checkbox`, `list`. |
| `text` | string | `""` | Caption / label. For `label` with a binding it prefixes the live value. |
| `binding` | string | `""` | Name in the binding table (see below). Empty means unbound. |
| `min` / `max` | number | `0` / `1` | Slider range. Writes are clamped to this range. |
| `anchor` | string | `"top-left"` | `top-left`, `top-right`, `bottom-left`, `bottom-right`, `center`, `stretch`. |
| `x` / `y` | number | `0` | Offset from the anchored position. Ignored when the parent stacks children. |
| `width` / `height` | number | derived | Explicit size; omit to use the per-type default (or to stretch when stacked). |
| `padding` | number | `0` | Inset applied to this panel's content rect. |
| `spacing` | number | `0` | Gap inserted between stacked children. |
| `direction` | string | `"column"` for panels | `row`, `column`, or `none` (children keep their own anchors). |
| `focusable` | bool | per type | Defaults to true for `button`, `slider`, `checkbox`, `list`. |
| `children` | array | `[]` | Nested widgets. |

### Anchors

| Anchor | Result |
| --- | --- |
| `top-left` | `x = frame.x + x`, `y = frame.y + y` |
| `top-right` | right-aligned inside the frame |
| `bottom-left` | bottom-aligned inside the frame |
| `bottom-right` | both |
| `center` | centred in the frame (plus `x`/`y` offset) |
| `stretch` | fills the frame (when `width`/`height` are omitted) |

### Stacking

If a widget has `direction: "row"` or `"column"` (panels default to `column`), its children are
placed sequentially in the content rect (`rect` inset by `padding`), separated by `spacing`.
Stacked children stretch across the content width unless they declare `width`. With
`direction: "none"` children position themselves using their own anchors.

### Default sizes

| Type | Default width × height |
| --- | --- |
| `panel` | 220 × 200 |
| `label` | 200 × 22 |
| `button` | 120 × 26 |
| `slider` | 180 × 22 |
| `checkbox` | 160 × 22 |
| `list` | 200 × 120 |

## Binding names

Bindings are resolved by `minieng::ui::BindingTable`
(`engine/include/minieng/ui/bindings.hpp`, registered by `register_simulation_bindings`).

| Binding | Kind | Reads / writes |
| --- | --- | --- |
| `physics.gravity.x` / `physics.gravity.y` | float (−20..20 / −30..10) | `PhysicsConfig::gravity` |
| `physics.restitution` | float (0..1) | selected body's restitution, else `default_restitution` |
| `physics.friction` | float (0..1) | selected body's friction, else `default_friction` |
| `physics.damping` | float (0..1) | `PhysicsConfig::linear_damping` |
| `physics.angular_damping` | float (0..1) | `PhysicsConfig::angular_damping` |
| `selected.position.x` / `.y` / `.z` | float (−50..50) | selected entity's local position |
| `selected.velocity.x` / `.y` / `.z` | float (−50..50) | selected entity's rigid-body velocity |
| `selected.mass` | float (0.1..100) | selected entity's mass |
| `sim.pause` | bool (toggle) | pauses the fixed loop |
| `sim.step` | action | advances exactly one fixed tick |
| `sim.reset` | action | rebuilds the scene from the current seed |
| `debug.wireframe` | bool (toggle) | clears the viewport background |
| `stats.fps`, `stats.ticks`, `stats.bodies`, `stats.pairs`, `stats.queries_per_sec` | read-only float | live statistics for HUD labels |

Unknown binding names are inert: the widget renders but reads 0 and writes are ignored. Read-only
bindings ignore writes.

## Focus and navigation

Focusable widgets are collected in depth-first order, which is also the Tab order.

- Arrow keys, `W`/`A`/`S`/`D` and the gamepad d-pad move focus to the nearest focusable widget in
  that direction (scored by distance along the movement axis plus a penalty for being off-axis).
- `Tab` / `Shift+Tab` walk the list in order.
- Every move wraps: pressing Right on the right-most widget lands on the left-most one, and Tab
  past the last widget returns to the first.
- `Enter`, `Space`, or the gamepad face-down button activates the focused widget: actions fire,
  checkboxes/`sim.pause` toggle.
- The focused widget is outlined in the app viewport.

## Adding a widget

Append to `widgets` (or to a panel's `children`) and relaunch:

```json
{
  "id": "bounce_speed",
  "type": "slider",
  "text": "Restitution",
  "binding": "physics.restitution",
  "min": 0, "max": 1,
  "focusable": true
}
```

Because the panel stacks its children, the new slider is placed automatically and the following
widgets shift down; no coordinates need to be written by hand.
