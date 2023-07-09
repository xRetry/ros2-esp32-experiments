import numpy as np
import json
import matplotlib.pyplot as plt

def load_data():
    with open('src/esp32_demo/data/vals.json', 'r') as f:
        data = json.load(f)

    return np.array(data['sent']), np.array(data['received']), data['ampl'], data['shift']

def main():
    vals_sent, vals_rec, ampl, shift = load_data()
    t_min = min([min(vals_sent[:, 0]), min(vals_rec[:, 0])])
    val_max = 1430
    val_min = 0
    vals_rec[:, 1] = 100 / (val_max - val_min) * vals_rec[:, 1] + val_min
    plt.figure()
    plt.plot(vals_rec[:, 0] - t_min, vals_rec[:, 1], label='Received signal with correction')
    plt.plot(vals_sent[:, 0] - t_min, vals_sent[:, 1], label='Sent signal')
    plt.xlabel('Time [s]')
    plt.ylabel('Amplitude')
    plt.legend()
    plt.show()
    

if __name__ == '__main__':
    main()
