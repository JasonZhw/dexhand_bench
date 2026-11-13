"""Optional observers consume snapshots; they never change task success or actions."""
from typing import Protocol
from .physics import contact_metrics


class DiagnosticProbe(Protocol):
    name: str
    requires_contacts: bool
    def reset(self, initial_state: dict) -> None: ...
    def observe(self, state: dict, transition: dict) -> None: ...
    def result(self) -> dict: ...


class ContactEnvelope:
    name = 'contact_envelope'
    requires_contacts = True

    def reset(self, initial_state):
        self.count = 0
        self.missing = 0
        self.penetration = None
        self.force = None

    def observe(self, state, transition):
        contacts = state.get('contacts')
        if contacts is None:
            self.missing += 1
            return
        metrics = contact_metrics(contacts)
        self.count += 1
        self.penetration = max(
            self.penetration or 0.,
            metrics['max_penetration_m'])
        self.force = max(self.force or 0., metrics['normal_force_sum_N'])

    def result(self):
        return {'samples_available': self.count, 'samples_unavailable': self.missing,
                'max_penetration_m': self.penetration, 'max_normal_force_sum_N': self.force,
                'interpretation': 'control-step-end contact diagnostics; not physical accuracy or net force'}
