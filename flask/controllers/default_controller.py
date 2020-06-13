
from flask import Flask, render_template, make_response

def mainpage() -> str:
    
    return make_response(render_template('login.html'))
