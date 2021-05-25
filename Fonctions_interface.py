import os , uuid
from time import sleep
from tkinter.filedialog import *
from tkinter import *
from Moteur_de_rendu_3d_par_rasterisation_vers_3 import *



def get_IP():
    return str(hex(uuid.getnode()))

def f_rendu(filepath,taille,rota_camera,depl_champ, depl_lamp, i_objet,v_puissance_lampe):
    lampe = (depl_lamp,(1,1,1),v_puissance_lampe)
    camera = ((1,1,1),rota_camera,(1,1,1),depl_champ)
    word =  lire_fichier2(filepath)
    nom_image = "image_rendu"
    image = (taille,taille,(100,100,100),nom_image)
    return Moteur_de_rendu_3d (word, camera, lampe, image, i_objet)

def ecrire_fichier2(chemin, data):
    with open(chemin, 'wb') as fichier:
        mon_pickler = Pickler(fichier)
        mon_pickler.dump(data)
        
def lire_fichier2(chemin):
    with open(chemin, 'rb') as fichier:
        mon_depickler = Unpickler(fichier)
        data = mon_depickler.load()
    return data

def sortie():
    exit(0)

    
