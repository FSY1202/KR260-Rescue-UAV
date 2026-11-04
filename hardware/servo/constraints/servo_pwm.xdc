## Candidate-only servo outputs; adapter H7 mapping supplied by owner.
## H7 pin 1 / GPIO19 -> Y13; H7 pin 2 / GPIO16 -> AB15.
set_property PACKAGE_PIN Y13 [get_ports servo0_pwm]
set_property PACKAGE_PIN AB15 [get_ports servo1_pwm]
set_property IOSTANDARD LVCMOS33 [get_ports {servo0_pwm servo1_pwm}]
set_property DRIVE 4 [get_ports {servo0_pwm servo1_pwm}]
set_property SLEW SLOW [get_ports {servo0_pwm servo1_pwm}]
