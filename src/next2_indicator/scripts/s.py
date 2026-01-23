#!/usr/bin/env python3

import time
from typing import List

from dmx import Colour, DMXInterface, DMXUniverse, DMXLight


class DMXLight4Slot(DMXLight):
    """DMX light with RGBW channels."""
    
    def __init__(self, address: int = 1):
        super().__init__(address=address)
        self._colour = Colour(0, 0, 0)
        self._white = 0

    @property
    def slot_count(self) -> int:
        return 4

    def set_colour(self, colour: Colour):
        self._colour = colour

    def set_white(self, white: int):
        self._white = max(0, min(int(white), 255))

    def serialise(self) -> List[int]:
        return self._colour.serialise() + [self._white]
