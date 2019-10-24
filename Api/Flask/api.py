from flask import Flask
import requests
from flask_restful import Resource, Api

app = Flask(__name__)
api = Api(app)

class Hello(Resource):				#Clase hello hereda de Resource (para saber qué ruta asocia a cada Clase)
    def get(self, name):
        return {"Hello":name}		#Llaves para asignar varios valores ej: "hello"

api.add_resource(Hello, '/hello/<name>')

if __name__ == '__main__':
 app.run(debug=True, host='0.0.0.0', port=5000)