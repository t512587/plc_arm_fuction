# Service and Flow lifecycle

All `plc2` services and flows expose the same lifecycle values:

| Status | Meaning | May start the next stage |
| --- | --- | --- |
| `pending` | Not started | No |
| `running` | Executing a command or precheck | No |
| `waiting_signal` | Waiting for PLC feedback | No |
| `success` | Completed successfully | Yes |
| `cancelled` | Cancelled | No |
| `timeout` | Timed out | No |
| `error` | Failed | No |

Detailed hardware activity remains in `step` and `state`. Code must use
`result.succeeded` or `result.status == LifecycleStatus.SUCCESS` to decide
whether the next stage may run. Do not parse `message` text.

```python
home_result = home_flow.run(cancel_event)
if not home_result.succeeded:
    return home_result

vision_result = vision_height_flow.run(cancel_event)
if not vision_result.succeeded:
    return vision_result
```

Every component exposes its latest snapshot:

```python
snapshot = lift_service.status_snapshot
print(snapshot.status.value, snapshot.step, snapshot.message)
```

The API returns final lifecycle status in each Flow response and exposes all
current component snapshots at:

```text
GET /lifecycle/status
```

Example Flow response:

```json
{
  "ok": true,
  "data": {
    "status": "success",
    "state": "success",
    "step": "move_to_height",
    "message": "視覺高度定位完成：560mm",
    "elapsed_seconds": 4.2,
    "height_mm": 560.0
  },
  "error": null
}
```

`cancel_requested: true` only means that the cancellation request was accepted.
Wait for the running Flow to return `status: cancelled` before starting another
stage.
