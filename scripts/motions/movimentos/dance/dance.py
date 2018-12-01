from Audio import Audio
from Movements import Movements
from Eyes import Eyes
from time import sleep

m = Movements()
a = Audio()
e = Eyes()

a.speak("Let's move it!")

sleep(2)
while True:
    a.play_mp3("/home/pi/beo/Examples/dance/arturMusic.mp3")
    e.set_expression("default")
    m.play_motion_file("part1")

    m.play_motion_file("part2")

    e.set_expression("shut")
    m.play_motion_file("part3")

    m.play_motion_file("part4")

    m.play_motion_file("part5")

    m.play_motion_file("slowmo")
    
    a.speak("Oh yeah!")
