
def make_header(ob_size):
    """Make the log header.

    This needs to be done dynamically because the observations used as input 
    to the NN may differ.
    """
    entries = []
    entries.append("t")
    for i in range(ob_size):
        entries.append("ob{}".format(i))
    for i in range(4):
        entries.append("ac{}".format(i))
    entries.append("p") # roll rate
    entries.append("q") # pitch rate
    entries.append("r") # yaw rate
    entries.append("p-sp") # roll rate setpoint
    entries.append("q-sp") # pitch rate setpoint
    entries.append("r-sp") # yaw rate setpoint
    for i in range(4):
        entries.append("y{}".format(i))
    for i in range(4):
        entries.append("w{}".format(i)) # ESC rpms
    entries.append("reward")

    return ",".join(entries)
