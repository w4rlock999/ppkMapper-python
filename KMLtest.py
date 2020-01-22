from os import path
from pykml import parser

kmlFile = path.join('./data/raw_201810252355.kml')

with open(kmlFile) as f:
    doc = parser.parse(f)


    