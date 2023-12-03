from flask import Flask, render_template, request,send_file
import board
import neopixel
import os

app = Flask(__name__)

settings = {'led_count': 0, 'rgb_values': {'red': 0, 'green': 0, 'blue': 0}}
connected_pin = board.D10
num_pixels = 30  # Default value
ORDER = neopixel.GRB
pixels = neopixel.NeoPixel(connected_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER)

UPLOAD_FOLDER = '/home/semubot/uploads'  # Folder to store uploaded files
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

@app.route('/')
def index():
    return render_template('index.html', settings=settings)


@app.route('/refresh_webpage')
def refresh_webpage():
    return render_template('webpage.html')

@app.route('/image')
def get_image():
    return send_file('/home/semubot/uploads/uploaded_image.jpg', mimetype='image/jpg')


@app.route('/', methods=['POST'])
def process_form():
    if 'red' in request.form and 'green' in request.form and 'blue' in request.form:
        settings['rgb_values']['red'] = int(request.form['red'])
        settings['rgb_values']['green'] = int(request.form['green'])
        settings['rgb_values']['blue'] = int(request.form['blue'])

    if 'led_count' in request.form:
        settings['led_count'] = int(request.form['led_count'])
        light_LED(settings['rgb_values']['red'], settings['rgb_values']['green'], settings['rgb_values']['blue'], settings['led_count'])

    # Handle file upload
    if 'image' in request.files:
        file = request.files['image']
        if file.filename != '':
            file_path = os.path.join(app.config['UPLOAD_FOLDER'], 'uploaded_image.jpg')
            file.save(file_path)
            print("Uploaded file:", file_path)

    return render_template('index.html', settings=settings)

def light_LED(R, G, B, pixel_count):
    print("R:", R, "G:", G, "B:", B, "Count:", pixel_count)
    pixels.fill((R, G, B))
    pixels.show()

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')