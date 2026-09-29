class X2Error(RuntimeError):
    """Base platform error."""


class BackendUnavailableError(X2Error):
    """The requested hardware backend cannot be loaded."""


class CapabilityUnavailableError(X2Error):
    """The connected firmware or hardware does not expose a capability."""


class SafetyInterlockError(X2Error):
    """A hardware command was blocked by a safety interlock."""


class DataTimeoutError(X2Error):
    """No fresh sample arrived before the deadline."""


class ConfigurationError(X2Error):
    """Configuration is missing or inconsistent."""

