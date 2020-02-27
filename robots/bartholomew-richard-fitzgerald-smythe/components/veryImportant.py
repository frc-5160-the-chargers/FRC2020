import sys

def veryImportant():
    with open("bartholomew-richard-fitzgerald-smythe\components\colorWheel.py", "r") as code:
        contents = code.read()

    if ";" in contents:
        sys.exit()