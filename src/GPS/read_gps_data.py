import pynmea2


with open('gps_data.txt',mode='r') as in_file, open('gps_output.txt', mode='w') as out_file:

    for line in in_file:
        try:
            msg = pynmea2.parse(line)
            print(repr('latitude: {} , longitude: {}'.format(msg.latitude, msg.longitude)))
        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            continue
