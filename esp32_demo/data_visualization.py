import numpy as np
import json
import matplotlib.pyplot as plt

def load_data(path):
    with open(path, 'r') as f:
        data = json.load(f)

    return np.array(data['sent']), np.array(data['received']), data['ampl'], data['shift']

def calibrate_data(vals_sent, vals_rec):
    t_min = min([min(vals_sent[:, 0]), min(vals_rec[:, 0])])
    val_max = 1430
    val_min = 0
    vals_rec[:, 1] = 100 / (val_max - val_min) * vals_rec[:, 1] + val_min

    vals_rec[:, 0] -= t_min
    vals_sent[:, 0] -= t_min
    return vals_sent, vals_rec

def main():

    rates = [
        ['2s', '500ms'], 
        ['50ms', '1ms']
    ]

    _, axs = plt.subplots(2, 2, figsize=(10, 6), sharex='all', sharey='all')
    for i in range(len(rates)):
        for j in range(len(rates[0])):
            vals_sent, vals_rec, _, _ = load_data(f'data/vals_{rates[i][j]}.json')
            vals_sent, vals_rec = calibrate_data(vals_sent, vals_rec)

            axs[i, j].plot(vals_rec[:, 0], vals_rec[:, 1], label='Received signal (calibrated)')
            axs[i, j].plot(vals_sent[:, 0], vals_sent[:, 1], label='Sent signal')
            if i == len(rates)-1:
                axs[i, j].set_xlabel('Time [s]')
            if j == 0:
                axs[i, j].set_ylabel('Amplitude')
            axs[i, j].set_title(f'Refresh Rate {rates[i][j]}')
            plt.gca().legend()
    plt.tight_layout()
    plt.show()
    

if __name__ == '__main__':
    main()
