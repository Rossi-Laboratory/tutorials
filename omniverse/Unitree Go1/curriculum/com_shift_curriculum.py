def update_com_shift(step_count):
    if step_count < 5000:
        return 0.01
    elif step_count < 15000:
        return 0.03
    else:
        return 0.05
