from flask import Flask, render_template, request
import board
import neopixel

app = Flask(__name__)

settings = {'led_count': 0, 'rgb_values': {'red': 0, 'green': 0, 'blue': 0}}
connected_pin = board.D18
num_pixels = 30  # Default value
ORDER = neopixel.GRB
pixels = neopixel.NeoPixel(connected_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER)

@app.route('/')
def index():
    return render_template('index.html', settings=settings)

@app.route('/', methods=['POST'])
def process_form():
    if 'red' in request.form and 'green' in request.form and 'blue' in request.form:
        settings['rgb_values']['red'] = int(request.form['red'])
        settings['rgb_values']['green'] = int(request.form['green'])
        settings['rgb_values']['blue'] = int(request.form['blue'])

    if 'led_count' in request.form:
        settings['led_count'] = int(request.form['led_count'])
        light_LED(settings['rgb_values']['red'], settings['rgb_values']['green'], settings['rgb_values']['blue'], settings['led_count'])

    return render_template('index.html', settings=settings)

def light_LED(R, G, B, pixel_count):
    global pixels
    pixels = neopixel.NeoPixel(connected_pin, pixel_count, brightness=0.2, auto_write=False, pixel_order=ORDER)
    pixels.fill((R, G, B))
    pixels.show()

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
