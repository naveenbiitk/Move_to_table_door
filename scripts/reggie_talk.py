# Import the required module for text 
# to speech conversion
from gtts import gTTS
  
# This module is imported so that we can 
# play the converted audio
import os
import playsound
# The text that you want to convert to audio
aruco_id=201

roomnumber= aruco_id
roomtext= "Hello, I am Reggie. Room {} your water delivery is here! Time to hydrate!".format(roomnumber)

# Language in which you want to convert
language = 'en'
  
# Passing the text and language to the engine, 
# here we have marked slow=False. Which tells 
# the module that the converted audio should 
# have a high speed
myobj = gTTS(text=roomtext, lang=language, slow=False)
 
# Saving the converted audio in a mp3 file named
# welcome
myobj.save("reggie_{}.mp3".format(roomnumber))
  
# Playing the converted file
playsound.playsound("/home/hello-robot/catkin_ws/src/find_fridge/scripts/reggie_{}.mp3".format(roomnumber), True)

