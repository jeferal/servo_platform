"""
    Some utilities for the scripts that command the ball controller.
"""


class EMAFilter:
    def __init__(self, alpha, initial_condition=0.0):
        self.alpha = alpha
        self.filtered_value = initial_condition

    def update(self, new_value) -> None:
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value

    def get_position(self):
        return self.filtered_value

    def set_alpha(self, alpha):
        self.alpha = alpha
