"""Helper script to calculate PID gains using the Ziegler-Nichols method"""
import argparse
def ziegler_nichols(Ku, Tu):
    Kp = 0.6 * Ku
    Ki = 1.2 * Ku/Tu
    Kd = 3 * Ku * Tu/40.0
    return Kp, Ki, Kd

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Calculate PID gains based on Ziegler–Nichols method")
    parser.add_argument('Ku', type=float, help="Ultimate gain.")
    parser.add_argument('Tu', type=float, help="Oscillation period.")
    args = parser.parse_args()

    Ku = args.Ku
    Tu = args.Tu
    Kp, Ki, Kd = ziegler_nichols(Ku, Tu)

    print ("Kp=", Kp)
    print ("Ki=", Ki)
    print ("Kd=", Kd)
