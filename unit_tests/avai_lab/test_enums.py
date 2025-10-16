# SPDX-License-Identifier: MIT
# Authors: Kevin Losing 

import pytest
from avai_lab.enums import YELLOW_CONE, BLUE_CONE, ORANGE_CONE, UNKNOWN_CONE, string_to_label

def test_cone_values():
    assert YELLOW_CONE == 1
    assert BLUE_CONE == 2
    assert ORANGE_CONE == 3
    assert UNKNOWN_CONE == -2

def test_string_to_label_mapping():
    assert string_to_label["yellow_cones"] == YELLOW_CONE
    assert string_to_label["blue_cones"] == BLUE_CONE
    assert string_to_label["orange_cones"] == ORANGE_CONE
