from flask import Flask
import requests
from flask_restful import Resource, Api

app = Flask(__name__)
api = Api(app)

@app.route('/hello/<name>')										#Añade el recurso
def hello(name):												#Crea la función dentro de la ruta
    info = requests.get('http://127.0.0.1:5000/hello/'+name)    #194 coords es a 21.88m-------11.28cm es a 0.000001 coords
    return info.text
    #return "asd"

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=80)