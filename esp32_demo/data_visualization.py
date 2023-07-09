import numpy as np
import json

def load_data():
    with open('src/esp32_demo/data/vals.json', 'r') as f:
        data = json.load(f)

    return data['sent'], data['received'], data['ampl'], data['shift']

def main():
    vals_sent, vals_rec, ampl, shift = load_data()
    

if __name__ == '__main__':
    main()
