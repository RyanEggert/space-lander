# dc_step_response.py
import time
from csv import DictWriter
from usb_vendor import PIC_USB
from os import getcwd

def main(start_vel, end_vel):
    test_len = 10  # Length of test in seconds.
    read_frq = 100  # Frequency of reads
    step_time = 1  # Time (in sec) at which to perform step.

    comms = PIC_USB(0x0005)  
    read_period = 1/float(read_frq)

    STEPPED = False
    motor_speed = start_vel[0]
    motor_dir = start_vel[1]
    print("STARTING STEP RESPONSE TEST")
    data = []
    print("\tStarting at speed {}, dir {}".format(motor_speed, motor_dir))
    comms.command_dcmotor(motor_speed, motor_dir)
    test_start_time = time.clock()
    prev_read_quad_time = test_start_time
    prev_quad_counter = comms.get_quad_info()["counter"]
    while time.clock() - test_start_time < test_len:
        read_start = time.clock()
        this_read = {}
        if read_start > step_time and not STEPPED:
            motor_speed = end_vel[0]
            motor_dir = end_vel[1]
            print("\tStepping to speed {}, dir {}".format(motor_speed, motor_dir))
            comms.command_dcmotor(motor_speed, motor_dir)
            STEPPED = True
        read_quad_time = time.clock()
        quad_info = comms.get_quad_info()
        this_read["time"] = read_quad_time
        this_read["motor_speed_cmd"] = motor_speed
        this_read["motor_dir_cmd"] = motor_dir
        this_read["quad_counter"] = quad_info["counter"]
        this_read["quad_overflow"] = quad_info["overflow"]
        this_read["quad_calc_speed"] = abs(quad_info["counter"] - prev_quad_counter) / (read_quad_time - prev_read_quad_time)
        data.append(this_read)
        prev_read_quad_time = read_quad_time
        prev_quad_counter = quad_info["counter"]
        while time.clock() - read_start < read_period:
            pass
    print("\tTest Concluded. Writing data...")

    filename = "step_response_{0[0]}-{0[1]}_to_{1[0]}-{1[1]}".format(start_vel, end_vel)
    headers = ["time", "motor_speed_cmd", "motor_dir_cmd", "quad_counter", "quad_overflow", "quad_calc_speed"]

    with open("data/{}.csv".format(filename), 'wb') as out_data:
        writer = DictWriter(out_data, fieldnames=headers)
        writer.writeheader()
        writer.writerows(data)

    print("\tData saved to \"data/{}.csv\"".format(filename))
    print("ENDING STEP RESPONSE TEST")


if __name__ == '__main__':
    start_vel = (0, 1)
    end_vel = (65535, 1)
    main(start_vel, end_vel)