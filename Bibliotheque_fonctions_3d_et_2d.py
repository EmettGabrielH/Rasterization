from PIL import Image
from numpy import dot,linalg, zeros, uint8, log, array
from math import cos, sin, tan, pi, floor, inf, sqrt, acos
from pickle import *

from sys import stdout

from time import time

"""
Global
"""
def lire_rendu():
    return rendu
def lire_rendu_objets():
    return rendu_objets
"""
Traitement Fichiers
"""
def ecrire_fichier(nom_fichier, data):
    with open("Objet 3d\{}".format(nom_fichier), 'wb') as fichier:
        mon_pickler = Pickler(fichier)
        mon_pickler.dump(data)
def lire_fichier(nom_fichier):
    with open("Objet 3d\{}".format(nom_fichier), 'rb') as fichier:
        mon_depickler = Unpickler(fichier)
        data = mon_depickler.load()
    return data

"""
Traitement Images
"""
def lire_image (nom_image):
    img = image.imread("{}.png".format(nom_image))
    if img.dtype == float32:
        img = (img * 255).astype(uint8)
    return img

def enregistrer_image(nom_image,data):
    im = Image.fromarray(data)
    im.save("{}.png".format(nom_image))
def afficher_image(data):
    rendu = Image.fromarray(data, 'RGB')
    rendu.show()

"""
Traitement Vidéos
"""
def creation_video (nom_fichier, liste_images,fps):
    path_img = "Rendus/"
    frame=imread(path_img+liste_images[0])
    h,l,c=frame.shape
    fourcc = VideoWriter_fourcc(*'XVID')
    out = VideoWriter('{}.mp4'.format(nom_fichier),fourcc, fps, (l,h))
    for i in range(len(liste_images)):
        frame=imread(path_img+liste_images[i])
        out.write(frame)
    out.release()

"""
Traitement mathematiques, matrices
"""
# 1 Traitement de matrices
def sinus_cosinus_angle (angle):
    angle = ((angle%360)/180)*pi
    sinus = sin(angle)
    cosinus = cos(angle)
    return sinus,cosinus

def matrice_Transformation (d,r,t):
    #d, r, t = dilatation ,rotation ,translation
    matrice_translation = ((1,0,0,0) , (0,1,0,0) , (0,0,1,0) , (t[0],t[1],t[2],1))
    matrice_dilatation  = ((d[0],0,0,0) , (0,d[1],0,0) , (0,0,d[2],0) , (0,0,0,1))
    
    s,c = sinus_cosinus_angle(r[0])
    matrice_Rx = ((1,0,0,0) , (0,c,-s,0) , (0,s,c,0) , (0,0,0,1))
    s,c = sinus_cosinus_angle(r[1])
    matrice_Ry = ((c,0,s,0) , (0,1,0,0) , (-s,0,c,0) , (0,0,0,1))
    s,c = sinus_cosinus_angle(r[2])
    matrice_Rz = ((c,-s,0,0) , (s,c,0,0) , (0,0,1,0) , (0,0,0,1))
    matrice_rotation = dot(dot(matrice_Rx,matrice_Ry), matrice_Rz)

    matrice_transformation = dot(dot(matrice_dilatation,matrice_rotation),matrice_translation )
    return matrice_transformation

def matrice_Perspective (fov, n, f):
    # fov, n, f = champ_vision, premier_plan, arriere_plan
    s = 1/(tan(((fov%360)/2)*(pi/180)))
    return ((s,0,0,0) , (0,s,0,0) , (0,0,-f/(f-n),-(f*n)/(f-n)) , (0,0,0,-1))
    
def local_vers_monde (point_local, matrice_local_vers_monde):
    point_monde = dot(point_local , matrice_local_vers_monde)
    return point_monde
def monde_vers_camera(point_monde,camera_local):
    point_camera = dot(camera_local, point_monde)
    return point_camera
def camera_vers_ecran(point_camera, matrice_perspective):
    point_ecran = dot(point_camera,matrice_perspective)
    return point_ecran[:-1]
def ecran_vers_grille(point_ecran,l,h):
    #l,h = largeur, hauteur de l'image
    point_grille = ( int(point_ecran[0]+l/2) , int(h-(point_ecran[1]+h/2)) )
    return point_grille
    
def procedure_local_vers_grille(point_local, matrice_local_vers_monde, camera_local, matrice_perspective, l, h):
    point_monde = local_vers_monde(point_local,matrice_local_vers_monde)
    point_camera = monde_vers_camera(point_monde,camera_local)
    point_ecran = camera_vers_ecran(point_camera, matrice_perspective)
    point_grille = ecran_vers_grille(point_ecran,l,h)
    return (point_grille[0],point_grille[1],point_ecran[2])

# 2 Visualisation
def initialisation_z_shader_rendu(image):
    global z_shader, rendu,rendu_objets
    
    l,h = image[0],image[1]
    z_shader = zeros((l,h,1),dtype=float)
    rendu = zeros((l,h,3), dtype=uint8)
    rendu_objets = zeros((l,h,1),dtype=int)
    
    for x in range(l):
        for y in range(h):
            z_shader[x,y] = -inf
            rendu[x,y] = image[2]
    return 1

def Algo_Bresenham(start, end, points, y_min):
    # Get line
    (x1, y1),(x2, y2) = start , end
    y1 , y2 = y1 - y_min, y2 - y_min
    dx, dy = (x2 - x1),(y2 - y1)
    is_steep = abs(dy) > abs(dx)
 
    if is_steep: (x1, y1),(x2, y2) = (y1, x1),(y2, x2) 
    
    if x1 > x2:  (x1, x2),(y1, y2) = (x2, x1), (y2, y1)
 
    dx, dy = (x2 - x1), (y2 - y1)
 
    steep,y = int(dx / 2.0), y1
    ystep = 1 if y1 < y2 else -1
    
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points[coord[1]]= min(coord[0],points[coord[1]][0]) , max(coord[0],points[coord[1]][1])
        steep -= abs(dy)
        if steep < 0:
            y += ystep
            steep += dx
    return points

def barycentre (A,B,C):
    x = (A[0]+B[0]+C[0])/3
    y = (A[1]+B[1]+C[1])/3
    z = (A[2]+B[2]+C[2])/3
    return (x, y, z)

def angle_surface_point (vecteur, liste_sommets, camera):
    s1,s2,s3 = liste_sommets
    # inclinaison du plan par rapport à la lampe (vecteur normal au plan par rapport au vecteur barycentre à lampe)
    B = s1 #B = barycentre(s1, s2, s3)
    vecteur_normal = produit_vectoriel_3d(s1, s2, s3)
    vecteur_normal_inverse = vecteur_inverse(vecteur_normal)
    
    if norme_vecteur_3d(vecteur_3d(camera , vecteur_somme(B,vecteur_normal))) < norme_vecteur_3d(vecteur_3d(camera , vecteur_somme(B,vecteur_normal_inverse))):
        vecteur_normal = vecteur_normal_inverse
        
    angle_point = angle_vecteurs(vecteur, vecteur_normal)
    return (angle_point % 0.5) * 2

def remplir_triangle (liste_sommets_grille, couleur, lampe,i_object):
    s1,s2,s3 = liste_sommets_grille
    
    aire = produit_vectoriel_2d(s1,s2,s3)
    if aire == 0 : return 1
    l, h = rendu.shape[0:2]
    
    y_min, y_max = min(s1[1],s2[1],s3[1]), max(s1[1],s2[1],s3[1])
    points = [[inf,-inf] for _ in range(y_max-y_min+1)]
    points = Algo_Bresenham(s1[0:2], s2[0:2],points,y_min)
    points = Algo_Bresenham(s1[0:2], s3[0:2],points,y_min)
    points = Algo_Bresenham(s2[0:2], s3[0:2],points,y_min)
    for y in range(y_min,y_max+1,1):
        x_min, x_max = points[y - y_min]
        for x in range(x_min,x_max+1,1):
            if  0<=x<l and 0<=y<h:
                s4 = (x,y)
                C1 = produit_vectoriel_2d(s2,s3,s4)/aire
                C2 = produit_vectoriel_2d(s3,s1,s4)/aire
                C3 = produit_vectoriel_2d(s1,s2,s4)/aire
                if C1>=0 and C2>=0 and C3>=0:
                    z = s1[2]*C1 + s2[2]*C2 + s3[2]*C3
                    point = (x,y,z)
                    if visible(point):
                        distance = norme_vecteur_3d(vecteur_3d((point),lampe))
                        rendu[x,y] = dot(couleur, log(distance+0.0001)/10)
                        rendu_objets[x,y] = i_object
    return 1


def anti_crenelage ():
    pass
def visible(point):
    if point[2] > z_shader[point[0],point[1]]:
        z_shader[point[0],point[1]] = point[2]
        return 1
    return 0
"""
Vecteurs
"""
def vecteur_2d (A,B):
    return (B[0]-A[0], B[1]-A[1])

def produit_vectoriel_2d(A, B, C):
    AB = vecteur_2d (A,B)
    AC = vecteur_2d (A,C)
    return (AB[0]*AC[1]) - (AC[0]*AB[1])

def vecteur_3d(A,B):
    return (B[0]-A[0], B[1]-A[1], B[2]-A[2])

def norme_vecteur_3d(AB):
    return sqrt(AB[0]**2 + AB[1]**2 + AB[2]**2)

def produit_scalaire_3d(A, B, C):
    AB = vecteur_3d (A,B)
    AC = vecteur_3d (A,C)
    return AB[0]*AC[0] + AB[1]*AC[1] + AB[2]*AC[2]

def produit_scalaire_3d_vecteur(AB,AC):
    return AB[0]*AC[0] + AB[1]*AC[1] + AB[2]*AC[2]

def produit_vectoriel_3d(A, B, C):
    AB = vecteur_3d (A,B)
    AC = vecteur_3d (A,C)
    return ( (AB[1]*AC[2]) - (AC[1]*AB[2]) , (AB[2]*AC[0]) - (AC[2]*AB[0]) , (AB[0]*AC[1]) - (AC[0]*AB[1]) )


def vecteur_inverse (AB):
    return (-AB[0],-AB[1],- AB[2])

def angle_vecteurs(AB, DC):
    return (acos (produit_scalaire_3d_vecteur(AB,DC) / (norme_vecteur_3d(AB) * norme_vecteur_3d(DC))) )  /(2*pi)

def vecteur_somme (AB, CD):
    return (AB[0]+CD[0], AB[1]+CD[1], AB[2]+CD[2])
       
"""
Autres
"""
def dilatation (point, Cx, Cy, Cz):
    # point = [x,y,z]
    return dot(point, ((Cx,0,0) , (0,Cy,0) , (0,0,Cz)))

def rotation(point, axe, angle):
    # point = [x,y,z]
    angle = ((angle%360)/180)*pi   # angle en radians
    s = sin(angle)
    c = cos(angle)
    if axe == 0:             #x
        return dot(point, ((1,0,0) , (0,c,-s), (0,s,c) ))
    elif axe == 1:           #y
        return dot(point, ((c,0,s) , (0,1,0) , (-s,0,c)))
    elif axe == 2:           #z
        return dot(point, ((c,-s,0), (s,c,0) , (0,0,1) ))

def translation(point , Tx, Ty, Tz):
    # point = [x,y,z,1]
    return dot(point, ((1,0,0,0) , (0,1,0,0) , (0,0,1,0) , (Tx,Ty,Tz,1)))

def equation_plan(A,B,C):
    a, b , c = produit_vectoriel_3d(A,B,C)
    d = -(a*A[0] + b*A[1] + c*A[2])
    return (a,b,c,d)
