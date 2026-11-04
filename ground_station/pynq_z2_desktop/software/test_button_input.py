import unittest
from button_input import ButtonPress


class ButtonPressTests(unittest.TestCase):
    def test_bounce_hold_release_and_second_press(self):
        button = ButtonPress(False, 0)
        events = []
        # Contact bounce, then a long hold, then release bounce and another press.
        samples = [(0.01, 1), (0.02, 0), (0.03, 1), (0.04, 0),
                   (0.05, 1), (0.08, 1), (0.10, 1), (0.2, 1), (5, 1),
                   (5.01, 0), (5.02, 1), (5.03, 0), (5.08, 0),
                   (5.2, 1), (5.25, 1), (6, 1)]
        for now, level in samples:
            if button.update(level, now):
                events.append(now)
        self.assertEqual(events, [0.10, 5.25])

    def test_held_during_boot_needs_release_first(self):
        button = ButtonPress(True, 0)
        self.assertFalse(button.update(True, 10))
        self.assertFalse(button.update(False, 11))
        self.assertFalse(button.update(False, 11.1))
        self.assertFalse(button.update(True, 12))
        self.assertTrue(button.update(True, 12.1))

    def test_short_noise_is_rejected(self):
        button = ButtonPress(False, 0)
        for i in range(100):
            self.assertFalse(button.update(i % 2, i*.01))
        self.assertFalse(button.update(False, 1))
        self.assertFalse(button.update(False, 2))


if __name__ == '__main__':
    unittest.main()
