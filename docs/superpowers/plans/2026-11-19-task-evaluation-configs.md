# Local task configuration and evaluation draft

Implement the user-approved scope: one existing JSON per task, shared evaluation routines, lightweight local tests. Keep simulation execution profiles and policy inference separate. Existing catalog/source-success fields remain intact.

1. Add benchmark specification blocks to all 21 task JSONs: scene and initial-state references, named scalar/boolean observations with units and definitions, completion/failure predicates, metrics, provenance and pending bindings. Retain source revisions; mark new subtask thresholds and episode aggregation as proposals.
2. Add a standard-library evaluator and CLI for recorded scalar observations. Validate configuration and input, preserve unknown values, enforce terminal/continuous-hold semantics and episode boundaries. Do not compute geometric helper booleans without an adapter or label synthetic traces as simulation evidence.
3. Add independent boundary/temporal/missing-data tests, full catalog audit, sample traces and an editable task overview. Run without simulator/GPU dependencies.

No source assets or policies are downloaded in this step. Original RoboDojo stage scores are documented separately; the new generic progress output is not advertised as a port of its transition score. Full source-score parity and live UniLab observation adapters remain pending.
