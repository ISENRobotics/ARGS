#!/usr/bin/python3
# -*- coding: utf-8 -*

import cgi

form = cgi.FieldStorage()
print("Content-type: text/html; charset=utf-8\n")

print(form.getvalue("name"))

html = """<!DOCTYPE html>
<head>
    <title>TurtleBot playing Tic tac toe</title>
</head>
<body>
    <p>
        <iframe width="620" height="380"
            src="http://192.168.2.127:8182/stream?topic=/augmented_reality_output/image_raw">
        </iframe>
    </p>
    <form action="/index.py" method="post">
        <input type="text" name="name" value="Votre nom" />
        <input type="submit" name="send" value="Envoyer information au serveur">
    </form>
</body>
</html>
"""
print(html)
