"""This module manage all audio related tasks from BEO
    Dependencies: gTTS, PyGame
"""

from time import sleep
from subprocess import call
from gtts import gTTS
from pygame import mixer, time
import pickle
import random
import os
import pwd

USERNAME = pwd.getpwuid(os.getuid()).pw_name

AUDIO_FOLDER = "/home/guichristmann/Programming/TCC_Beo_Game/Beo_Audio/audio_files/"
AUDIO_DATABASE = AUDIO_FOLDER+'database.dat'

def gera_nome():
    s = ''
    for i in range(15):
        s = s + chr(random.randint(ord('A'),ord('Z')))
    return s

class Audio(object):
    """ Audio class for BEO"""
    def __init__(self):
        try:
            f = open(AUDIO_DATABASE, 'rb')
            d = pickle.load(f)
            f.close()
            self.database = d
        except:
            self.database = {}

    def play_mp3(self, file, volume=1):
        """
        Description: Plays an mp3 file
        Utilization: play_mp3(file)
        Parameters:
         -file: mp3 filename
         -volume: volume value between 0.0 and 1.0(default)
        """

        mixer.init()
        mixer.music.set_volume(volume)
        mixer.music.load(file)
        mixer.music.play(loops=0)
        while mixer.music.get_busy():
            time.delay(100)
#        print('falado')
#        mixer.music.play()
#       while mixer.music.get_busy():
#            pass

    def say_number(self, number):
        """
        Description: Plays a number digit mp3 file stored at "/audio/audio/"
        Utilization: say_number(number)
        Parameters:
         -number: plays number digit mp3 file
        """
        for digit in number:
            self.play_mp3(AUDIO_FOLDER+digit+'.mp3')

    def gera_mp3(self, arquivo, texto):
        """
        Description: Generates an mp3 audio in Portuguese
        Utilization: gera_mp3(arquivo,texto)
        Parameters:
         -arquivo: filename for the generated mp3 file
         -texto: text to generate mp3 file
        """
        print('Gerando som de "'+texto+'"')
        tts = gTTS(text=texto, lang='pt')
        print("Salvando "+arquivo)
        tts.save(arquivo)

    def generate_mp3(self, arquivo, texto):
        """
        Description: Generates an mp3 audio in English
        Utilization: gera_mp3(arquivo,texto)
        Parameters:
         -arquivo: filename for the generated mp3 file
         -texto: text to generate mp3 file
        """
        print('Generating sound of "'+texto+'"')
        tts = gTTS(text=texto, lang='en-us')
        print('Saving '+arquivo)
        tts.save(arquivo)

    def gera_mp3_es(self, arquivo, texto):
        """
        Description: Generates an mp3 audio in Spanish
        Utilization: gera_mp3(arquivo,texto)
        Parameters:
         -arquivo: filename for the generated mp3 file
         -texto: text to generate mp3 file
        """
        print('Gerando som de "'+texto+'"')
        tts = gTTS(text=texto, lang='es-ES')
        print("Salvando "+arquivo)
        tts.save(arquivo)
    def gera_mp3_fr(self, arquivo, texto):
        """
        Description: Generates an mp3 audio in French
        Utilization: gera_mp3(arquivo,texto)
        Parameters:
         -arquivo: filename for the generated mp3 file
         -texto: text to generate mp3 file
        """
        print('Gerando som de "'+texto+'"')
        tts = gTTS(text=texto, lang='fr')
        print("Salvando "+arquivo)
        tts.save(arquivo)

    def gera_mp3_de(self, arquivo, texto):
        """
        Description: Generates an mp3 audio in German
        Utilization: gera_mp3(arquivo,texto)
        Parameters:
         -arquivo: filename for the generated mp3 file
         -texto: text to generate mp3 file
        """
        print('Gerando som de "'+texto+'"')
        tts = gTTS(text=texto, lang='de')
        print("Salvando "+arquivo)
        tts.save(arquivo)

    def gera_mp3_ja(self, arquivo, texto):
        """
        Description: Generates an mp3 audio in Japanese
        Utilization: gera_mp3(arquivo,texto)
        Parameters:
         -arquivo: filename for the generated mp3 file
         -texto: text to generate mp3 file
        """
        print('Gerando som de "'+texto+'"')
        tts = gTTS(text=texto, lang='ja')
        print("Salvando "+arquivo)
        tts.save(arquivo)

    def ajusta_mp3(self, origem, destino):
        """
        Description: Modifies echo, pitch and speed of an mp3 audio to portuguese language
        Utilization: ajusta_mp3(origem, destino)
        Parameters:
         -origem: filename of the mp3 file to be modified
         -destino: filename for the modified mp3 file
        """
        print('Ajustando')
        print(destino)
        call(['sox', origem, destino,\
            'pitch', '150', 'speed', '1.25',\
            'echo', '0.8', '0.88', '60', '0.4'])

    def ajusta_mp3_de(self, origem, destino):
        """
        Description: Modifies echo, pitch and speed of an mp3 audio to german lanaguage
        Utilization: ajusta_mp3_de(origem, destino)
        Parameters:
         -origem: filename of the mp3 file to be modified
         -destino: filename for the modified mp3 file
        """
        print('Ajustando')
        call(['sox', origem, destino,\
            'pitch', '+175', 'speed', '1.2',\
            'echo', '0.8', '0.88', '60', '0.4'])

    def ajusta_mp3_es(self, origem, destino):
        """
        Description: Modifies echo, pitch and speed of an mp3 audio to Spanish language
        Utilization: ajusta_mp3_de(origem, destino)
        Parameters:
         -origem: filename of the mp3 file to be modified
         -destino: filename for the modified mp3 file
        """
        print('Ajustando')
        call(['sox', origem, destino,\
            'pitch', '500', 'speed', '1.1',\
            'echo', '0.8', '0.88', '60', '0.4'])

    def adjust_mp3(self, origem, destino):
        """
        Description: Modifies echo, pitch and speed of an mp3 audio to english language
        Utilization: adjust_mp3(origem, destino)
        Parameters:
         -origem: filename of the mp3 file to be modified
         -destino: filename for the modified mp3 file
        """
        print('Adjusting')
        call(['sox', origem, destino,\
            'pitch', '+175', 'speed', '1.2',\
            'echo', '0.8', '0.88', '60', '0.4'])

    def speak(self, sentence):
        """
        Description: Generates an English audio, adjust it and plays the modified audio.
                     Don't save both non-modified and modified audio.
        Utilization: speak(sentence)
        Parameters:
         -sentence: text to be speak
        """
        self.fala(sentence, lang='en')


    def fala(self, frase, lang='pt'):
        """
        Description: Generates a Portuguese audio, adjust it and plays the modified audio.
                     Don't save both non-modified and modified audio.
        Utilization: speak(sentence,lang='pt')
        Parameters:
         -sentence: text to be speak
         -lang: Language of the sentence (default=pt)
        """
        #try:
        if True:
            if frase not in self.database:
                #print('Nova frase! Estou aprendendo!')
                nome = AUDIO_FOLDER+gera_nome()
                original = nome + '_orig.mp3'
                final = nome+'.mp3'
                if lang == 'pt':
                    self.gera_mp3(original, frase)
                    self.ajusta_mp3(original, final)
                else:
                    self.generate_mp3(original, frase)
                    self.adjust_mp3(original, final)
                call(['rm', '-f', original])
                self.database[frase] = final
                f = open(AUDIO_DATABASE, 'wb')
                pickle.dump(self.database, f)
                f.close()
            else:
                #print('Encontrei na base de dados essa frase!')
                final = self.database[frase]
            #print(final)
            self.play_mp3(final)
        #except:
        #    print("Failed TTS")
    def habla(self, frase):
        """
        Description: Generates a Portuguese audio, adjust it and plays the modified audio.
                     Don't save both non-modified and modified audio.
        Utilization: speak(sentence,lang='pt')
        Parameters:
         -sentence: text to be speak
         -lang: Language of the sentence (default=pt)
        """
        nome = "frase"
        original = nome + '_orig.mp3'
        final = nome+'.mp3'
        try:
            print("Try")
            print("gerando mp3")
            self.gera_mp3_es(original, frase)
            print("ajusta mp3")
            self.ajusta_mp3(original, final)
            self.play_mp3(final)
            call(['rm', '-f', original,\
                 final])
        except:
            print("Failed TTS")
