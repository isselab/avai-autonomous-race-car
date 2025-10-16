#!/usr/bin/env python3
"""
Enums for cone labels.
"""

# Cone label definitions
YELLOW_CONE = 1
BLUE_CONE = 2
ORANGE_CONE = 3
UNKNOWN_CONE = -2

# Mapping from class string to integer label
string_to_label = {
    "yellow_cones": YELLOW_CONE,
    "blue_cones": BLUE_CONE,
    "orange_cones": ORANGE_CONE
}
