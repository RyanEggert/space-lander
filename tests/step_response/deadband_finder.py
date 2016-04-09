# dc_step_response.py
import time
from csv import DictWriter
from usb_vendor import PIC_USB


def main(start_spd, end_spd, steps, direction):
    """
    Performs a motor deadband test, prints the result, and saves the data to a csv.
    """
    read_frq = 300  # Frequency of reads
    settle_period = 2 # Specify time (s) to let motor settle before assessing deadband
    moving_threshold = 10  # Number of ticks/second to consider the motor to have left the deadband

    comms = PIC_USB(0x0005)  
    read_period = 1/float(read_frq)

    motor_speed = start_spd
    motor_dir = direction
    data = []
    prev_read_quad_time = time.clock()
    prev_quad_counter = comms.get_quad_info()["counter"]
    speeds = range(start_spd, end_spd, steps)
    for spd in speeds:
        set_start = time.clock()
        print("Setting motor speed to {}.".format(spd))
        comms.command_dcmotor(spd, motor_dir)
        motor_speed = spd

        while time.clock() - set_start < settle_period:
            this_read = {}
            read_start = time.clock()
            quad_info = comms.get_quad_info()
            this_read["time"] = read_start
            this_read["motor_speed_cmd"] = motor_speed
            this_read["motor_dir_cmd"] = motor_dir
            this_read["quad_counter"] = quad_info["counter"]
            this_read["quad_overflow"] = quad_info["overflow"]
            this_read["quad_calc_speed"] = abs(quad_info["counter"] - prev_quad_counter) / (read_quad_time - prev_read_quad_time)
            data.append(this_read)
            prev_read_quad_time = read_start
            prev_quad_counter = quad_info["counter"]
            while time.clock() - read_start < read_period:
                pass
        # After motor has "settled"
        settle_check_time = time.clock()
        quad_info = comms.get_quad_info()
        settle_vel = abs(quad_info["counter"] - prev_quad_counter) / (settle_check_time - prev_read_quad_time)      
        if settle_vel > moving_threshold:
            print("Deadband end identified at speed {}.".format(spd))
            break

    print("\tTest Concluded. Writing data...")

    filename = "deadband_{0}-{1}_steps{2}_{3}".format(start_spd, end_spd, steps, direction)
    headers = ["time", "motor_speed_cmd", "motor_dir_cmd", "quad_counter", "quad_overflow", "quad_calc_speed"]

    with open("data/{}.csv".format(filename), 'wb') as out_data:
        writer = DictWriter(out_data, fieldnames=headers)
        writer.writeheader()
        writer.writerows(data)

    print("\tData saved to \"data/{}.csv\"".format(filename))
    print("ENDING DEADBAND TEST")


if __name__ == '__main__':
    # Vels are defined as (speed [0-65535], dir [0-1])

    main(500, 8000, 100, 1)