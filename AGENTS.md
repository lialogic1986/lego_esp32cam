# LEGO ESP32-CAM Project Notes

This repository is part of the LEGO Train System.

Before making architecture, protocol, or integration changes, read and follow
the shared project context in `../lego_project_context/`, especially:

* `AGENTS.md`
* `01_architecture.md`
* `02_services.md`
* `03_protocols.md`
* `07_decisions.md`

Core constraints:

* Follow the event-driven architecture.
* Do not break service boundaries.
* Use the `train_id` abstraction rather than `device_id` in system-level logic.
* Keep BLE provisioning stateless and full on each start.
* Preserve existing ports and protocols.
* Avoid blocking calls in async code.
* Prefer simple, robust solutions.
