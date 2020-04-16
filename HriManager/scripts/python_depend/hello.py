#/usr/bin/env python
# -*- coding:utf-8 -*-

from flask import Flask, request, make_response, render_template
app = Flask(__name__)

from PIL import Image
from StringIO import StringIO

# @app.route('/image')
# def genere_image():
#     mon_image = StringIO()
#     Image.new("RGB", (300,300), "#92C41D").save(mon_image, 'BMP')
#     reponse = make_response(mon_image.getvalue())
#     reponse.mimetype = "image/bmp"  # à la place de "text/html"
#     return reponse

# @app.route('/coucou')
# def dire_coucou():
#     return 'Coucou !'

# @app.route('/')
# def racine():
#     return "Le chemin de 'racine' est : " + request.path

# @app.route('/la')
# def ici():
#     return "Le chemin de 'ici' est : " + request.path

# @app.route('/contact', methods=['GET', 'POST'])
# def contact():
#     if request.method == 'GET':
#         # afficher le formulaire
#         return 'Coucou !'
#     else:
#         return 'Coucou11 !'
#         # traiter les données reçues
#         # afficher : "Merci de m'avoir laissé un message !"


# if __name__ == '__main__':
#     app.run(debug=True)

#! /usr/bin/python
# -*- coding:utf-8 -*-

# from flask import Flask
# app = Flask(__name__)

@app.route('/')
def accueil():
    # mots = ["bonjour", "à", "toi,", "visiteur."]
    return render_template('index.html')

if __name__ == '__main__':
    app.run(debug=True)
