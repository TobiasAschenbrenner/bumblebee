"""Flight routines available in the terminal selector."""

from bumblebee.flights.flip import FLIGHT as FLIP
from bumblebee.flights.hover import FLIGHT as HOVER

AVAILABLE_FLIGHTS = (HOVER, FLIP)
FLIGHTS_BY_KEY = {flight.key: flight for flight in AVAILABLE_FLIGHTS}


__all__ = ["AVAILABLE_FLIGHTS", "FLIGHTS_BY_KEY"]
