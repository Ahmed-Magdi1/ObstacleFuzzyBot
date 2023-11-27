from machine import Pin, utime_pulse_us
import numpy as np
import skfuzzy.control as ctrl
import skfuzzy as fuzz
import utime
from obstacle_avoidance import ultrasonic as us

# Define linguistic variables
left_ultrasonic = ctrl.Antecedent(np.arange(0, 400, 50), 'left_ultrasonic_reading')
front_ultrasonic = ctrl.Antecedent(np.arange(0, 400, 50), 'front_ultrasonic_reading')
right_ultrasonic = ctrl.Antecedent(np.arange(0, 400, 50), 'right_ultrasonic_reading')
left_motor = ctrl.Consequent(np.arange(-200, 200, 50), 'left_motor_RPM')
right_motor = ctrl.Consequent(np.arange(-200, 200, 50), 'right_motor_RPM')

# Membership functions for left_ultrasonic
left_ultrasonic['near'] = fuzz.trapmf(left_ultrasonic_reading.universe, [2, 2, 50, 150])
left_ultrasonic['medium'] = fuzz.trimf(left_ultrasonic_reading.universe, [100, 200, 300])
left_ultrasonic['far'] = fuzz.trapmf(left_ultrasonic_reading.universe, [250, 350, 400, 400])

# Membership functions for front_ultrasonic
front_ultrasonic['near'] = fuzz.trapmf(front_ultrasonic_reading.universe, [2, 2, 50, 150])
front_ultrasonic['medium'] = fuzz.trimf(front_ultrasonic_reading.universe, [100, 200, 300])
front_ultrasonic['far'] = fuzz.trapmf(front_ultrasonic_reading.universe, [250, 350, 400, 400])

# Membership functions for right_ultrasonic
right_ultrasonic['near'] = fuzz.trapmf(right_ultrasonic_reading.universe, [2, 2, 50, 150])
right_ultrasonic['medium'] = fuzz.trimf(right_ultrasonic_reading.universe, [100, 200, 300])
right_ultrasonic['far'] = fuzz.trapmf(right_ultrasonic_reading.universe, [250, 350, 400, 400])

# Membership functions for left_motor
left_motor['-ve_fast'] = fuzz.trapmf(left_motor_RPM.universe, [-200, -200, 45, 90])
left_motor['zero'] = fuzz.trimf(left_motor_RPM.universe, [-90, 0, 90])
left_motor['+ve_slow'] = fuzz.trapmf(left_motor_RPM.universe, [76.4, 76.4, 104, 159])
left_motor['+ve_fast'] = fuzz.trapmf(left_motor_RPM.universe, [117.5, 172.5, 200, 200])

# Membership functions for right_motor
right_motor['-ve_fast'] = fuzz.trapmf(right_motor_RPM.universe, [-200, -200, 45, 90])
right_motor['zero'] = fuzz.trimf(right_motor_RPM.universe, [-90, 0, 90])
right_motor['+ve_slow'] = fuzz.trapmf(right_motor_RPM.universe, [76.4, 76.4, 104, 159])
right_motor['+ve_fast'] = fuzz.trapmf(right_motor_RPM.universe, [117.5, 172.5, 200, 200])

# Fuzzy rules
rule1 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['near'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule2 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['near'] & right_ultrasonic['medium'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule3 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['near'] & right_ultrasonic['far'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule4 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['medium'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule5 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['medium'] & right_ultrasonic['medium'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule6 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['medium'] & right_ultrasonic['far'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule7 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['far'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule8 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['far'] & right_ultrasonic['medium'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule9 = ctrl.Rule(left_ultrasonic['near'] & front_ultrasonic['far'] & right_ultrasonic['far'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule10 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['near'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule11 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['near'] & right_ultrasonic['medium'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule12 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['near'] & right_ultrasonic['far'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule13 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['medium'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule14 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['medium'] & right_ultrasonic['medium'], left_motor['-ve_fast'], right_motor['+ve_slow'])
rule15 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['medium'] & right_ultrasonic['far'], left_motor['+ve_fast'], right_motor['+ve_slow'])
rule16 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['far'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule17 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['far'] & right_ultrasonic['medium'], left_motor['+ve_fast'], right_motor['+ve_slow'])
rule18 = ctrl.Rule(left_ultrasonic['medium'] & front_ultrasonic['far'] & right_ultrasonic['far'], left_motor['+ve_fast'], right_motor['+ve_slow'])
rule19 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['near'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule20 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['near'] & right_ultrasonic['medium'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule21 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['near'] & right_ultrasonic['far'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule22 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['medium'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule23 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['medium'] & right_ultrasonic['medium'], left_motor['+ve_slow'], right_motor['+ve_fast'])
rule24 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['medium'] & right_ultrasonic['far'], left_motor['+ve_fast'], right_motor['+ve_slow'])
rule25 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['far'] & right_ultrasonic['near'], left_motor['-ve_fast'], right_motor['-ve_fast'])
rule26 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['far'] & right_ultrasonic['medium'], left_motor['+ve_slow'], right_motor['+ve_fast'])
rule27 = ctrl.Rule(left_ultrasonic['far'] & front_ultrasonic['far'] & right_ultrasonic['far'], left_motor['+ve_fast'], right_motor['+ve_fast'])

# Control system
obstacle_avoidance = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20, rule21, rule22, rule23, rule24, rule25, rule26, rule27])
avoidance_decision = ctrl.ControlSystemSimulation(obstacle_avoidance)

# Test
sensor_distance = us.get_distance()
avoidance_decision.input['distance'] = sensor_distance
avoidance_decision.compute()

print(avoidance_decision.output['action'])
