"""Application-specific exceptions."""


class SafetyError(RuntimeError):
    """Raised when a flight condition is unsafe or ambiguous."""
