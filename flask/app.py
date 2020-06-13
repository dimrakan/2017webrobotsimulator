#!/usr/bin/env python3
import connexion
from flask_cors import CORS
from flask_pymongo import PyMongo

app = connexion.FlaskApp(__name__, specification_dir='./swagger/')
app.add_api('swagger.yaml', arguments={'title': 'This is a sample server Pe'})
app.debug = True
CORS(app.app)
app.app.config['SECRET_KEY']='sadad'
application=app.app

if __name__ == '__main__':
    app.run(threaded="True",host='0.0.0.0', port=8000)

