from time import sleep, time
from tkinter.filedialog import *
from tkinter import *
from PIL import Image
from numpy import dot,linalg, zeros, uint8, log, array
from math import cos, sin, tan, pi, floor, inf, sqrt, acos
from pickle import *


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



def cube (O,C,word):
    liste_points, liste_triangles, list_objets = word
    l_p,l_tri, l_obj = len(liste_points)-1,len(liste_triangles)-1,len(list_objets)-1
    c = C/2
    C_1, C_2, C_3,C_4, C_5, C_6 = (100,0,100),(0,0,255),(255,0,0),(0,255,0),(0,0,255),(255,255,255)
    A = (0-c,0-c,0-c)
    B = (0-c,0+c,0-c)
    C = (0-c,0-c,0+c)
    D = (0-c,0+c,0+c)
    E = (0+c,0-c,0-c)
    F = (0+c,0+c,0-c)
    G = (0+c,0-c,0+c)
    H = (0+c,0+c,0+c)
    liste_points.extend((A,B,C,D,E,F,G,H))
    ABC,BCD,EFG,FGH,ABE,BEF,BDF,DFH,DCH,CHG,CEG,CEA = (l_p+1,l_p+2,l_p+3),(l_p+2,l_p+3,l_p+4),(l_p+5,l_p+6,l_p+7), (l_p+6,l_p+7,l_p+8), (l_p+1,l_p+2,l_p+5), (l_p+2,l_p+5,l_p+6), (l_p+2,l_p+4,l_p+6), (l_p+4,l_p+6,l_p+8), (l_p+4,l_p+3,l_p+8), (l_p+3,l_p+8,l_p+7), (l_p+3,l_p+5,l_p+7), (l_p+3,l_p+5,l_p+1)
    liste_triangles.extend([(ABC,C_1),(BCD,C_1),(EFG,C_2) ,(FGH,C_2), (ABE,C_3), (BEF,C_3), (BDF,C_4), (DFH,C_4), (DCH,C_5), (CHG,C_5), (CEG,C_6), (CEA,C_6)])
    list_objets.extend([ ((l_tri+1,l_tri+2,l_tri+3,l_tri+4,l_tri+5,l_tri+6,l_tri+7,l_tri+8,l_tri+9,l_tri+10,l_tri+11,l_tri+12) ,((1,1,1),(0,0,0),O)) ])
    return (liste_points,liste_triangles,list_objets)
def triangle (A,B,C,word):
    liste_points, liste_triangles, list_objets = word
    l_p,l_tri, l_obj = len(liste_points)-1,len(liste_triangles)-1,len(list_objets)-1
    
    C_1 = (0,0,100)
    liste_points.extend((A,B,C))
    ABC= (l_p+1,l_p+2,l_p+3)
    liste_triangles.extend([(ABC,C_1)])
    list_objets.extend([ ([l_tri+1] ,((1,1,1),(0,0,0),(0,0,0))) ])
    return (liste_points,liste_triangles,list_objets)
def carre(O,C,word):
    liste_points, liste_triangles, list_objets = word
    l_p,l_tri, l_obj = len(liste_points)-1,len(liste_triangles)-1,len(list_objets)-1
    c = C/2
    C_1 = (0,100,0)
    A = (0-c,0-c,0-c)
    B = (0-c,0+c,0-c)
    C = (0-c,0-c,0+c)
    D = (0-c,0+c,0+c)
    liste_points.extend((A,B,C,D))
    ABC,BCD= (l_p+1,l_p+2,l_p+3),(l_p+2,l_p+3,l_p+4)
    liste_triangles.extend([(ABC,C_1),(BCD,C_1)])
    list_objets.extend([ ((l_tri+1,l_tri+2) ,((1,1,1),(0,0,0),O)) ])
    return (liste_points,liste_triangles,list_objets)

def Moteur_de_rendu_3d (word, camera, lampe, image, i_objet_selectionne):
    """
    objet3d: (liste_points, liste_triangles)
    camera : (dc,rc,tc,fov,proche,lointain)
    image : (largeur,hauteur,fond,fichier_sortie)
    """
    dil = 1.1
    l,h = image[0],image[1]
    liste_points, liste_triangle, liste_objet= word
    lampe, vecteur_lampe, intensite_lampe  = lampe
    
    camera_local = linalg.inv(matrice_Transformation ((1,1,1), camera[1], camera[2]))
    matrice_perspective = matrice_Perspective(camera[3][0],camera[3][1],camera[3][2])
    initialisation_z_shader_rendu(image)
    
    lampe = procedure_local_vers_grille(lampe+(1,),((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1)),camera_local, matrice_perspective,l,h)
    camera_coordonnees = dot(camera_local[0:3,0:3],(1,1,1))
    for i_objet,objet in enumerate(liste_objet):
        for index_triangle in objet[0]:
            triangle = liste_triangle[index_triangle][0]
            
            d,r,t = objet[1]
            matrice_local_vers_monde = matrice_Transformation(d,r,t)
            liste_sommets_grille ,liste_sommets =  [0,0,0], [0,0,0]
            for i in range(3):
                
                index_point = triangle[i]
                coord_point = liste_points[index_point]+(1,)
                point_grille = procedure_local_vers_grille(coord_point,matrice_local_vers_monde,camera_local, matrice_perspective,l,h)
                liste_sommets_grille[i] = (point_grille)
                liste_sommets[i] = coord_point
            
            angle_lampe = angle_surface_point(vecteur_lampe, liste_sommets, camera_coordonnees)
            couleur = dot(liste_triangle[index_triangle][1],angle_lampe*intensite_lampe)
            if i_objet+1 == i_objet_selectionne:
                couleur = dot(couleur, 1.6)
            
            remplir_triangle (liste_sommets_grille ,couleur, lampe,i_objet+1)

    anti_crenelage () 
    enregistrer_image(image[3],lire_rendu())
    return lire_rendu_objets()


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

def f_image():
    image.putpixel((x,y),(r,v,b))

def main():
    fenetre = Tk()
    fenetre.geometry("900x700")
    fenetre.title('Moteur de rendu')
    fenetre.iconbitmap('icone.ico')
    fenetre.resizable(width=False,height=False)

    global cadre, cadre_photo, filepath, i_objet, etat, TAILLE
    TAILLE = 480
    cadre = Frame(fenetre, width=1500, height=1500, borderwidth=1)
    cadre_photo = Frame(cadre, width=1500, height=1500, borderwidth=1)
    filepath = "#?"
    i_objet = [0]
    etat = 0
    def Interface_accueil():
        global cadre
        
        destructeur()
        
        Label(cadre, text="Moteur de rendu 3d par rastérisation", fg="black", font=("Times",23,"bold"),background="yellow").place(x=150, y=200)
        Label(cadre, text="Créateur: Emett Haddad", fg="black").place(x=310, y=300)
        Label(cadre, text="Crée a la date du 16/10/2020", fg="black").place(x=300, y=320)

    def Simulateur():
        global cadre, etat, filepath, depl_x, depl_y, depl_z, lamp_x, lamp_y, lamp_z, selec_taill , rota_x,rota_y,rota_z, i_objet, puissance_lampe

        
        destructeur()
        i_objet = [0]

        etat = 1
        
        if filepath == "#?":
            Ouvrir_fichier()
        Label(cadre, text="Simulateur", fg="black", font=("Times",23,"bold"),background="yellow").pack()
        Label(cadre, text="Fichier : {}".format(filepath), fg="black").pack()
        Label(cadre, text="-----------------------", fg="black").pack()
        Label(cadre, text="Créateur : Emett Haddad ©".format(filepath), fg="black").pack(side = BOTTOM)
        Label(cadre, text="-----------------------", fg="black").pack(side = BOTTOM)

        label_commandes = LabelFrame(cadre, text="Commandes", padx=20, pady=20)
        label_commandes.pack(side = RIGHT, expand="yes")
     
        Label(label_commandes).pack()
        Label(label_commandes, text="Rotation camera: (x,y,z)", fg="black").pack()
        var4 = StringVar(label_commandes) 
        rota_x = Spinbox(label_commandes, from_=0, to=360, textvariable=var4, command = mise_a_jour)
        var4.set(45)
        rota_x.pack()
        var5 = StringVar(label_commandes) 
        rota_y = Spinbox(label_commandes, from_=0, to=360, textvariable=var5, command = mise_a_jour)
        var5.set(45)
        rota_y.pack()
        var6 = StringVar(label_commandes) 
        rota_z = Spinbox(label_commandes, from_=0, to=360, textvariable=var6, command = mise_a_jour)
        var6.set(45)
        rota_z.pack()
        
        Label(label_commandes, text=" ", fg="black").pack()

        Label(label_commandes, text="Champ de vision: (x,y,z)", fg="black").pack()
        
        var = StringVar(label_commandes) 
        depl_x = Spinbox(label_commandes, from_=1, to=1000, textvariable=var, command = mise_a_jour)
        var.set(40)
        depl_x.pack()
        
        depl_y = Spinbox(label_commandes, from_=1, to=1000, command = mise_a_jour)
        depl_y.pack()
        depl_z = Spinbox(label_commandes, from_=100, to=1000, command = mise_a_jour)
        depl_z.pack()
        
        Label(label_commandes, text=" ", fg="black").pack()
        
        Label(label_commandes, text="Position lampe : (x,y,z)", fg="black").pack()
        lamp_x = Spinbox(label_commandes, from_=0, to=360, command = mise_a_jour)
        lamp_x.pack()
        lamp_y = Spinbox(label_commandes, from_=0, to=360, command = mise_a_jour)
        lamp_y.pack()
        lamp_z = Spinbox(label_commandes, from_=0, to=360, command = mise_a_jour)
        lamp_z.pack()

        Label(label_commandes, text=" ", fg="black").pack()
        
        Label(label_commandes, text="Puissance lampe :", fg="black").pack()
        var2 = StringVar(label_commandes)
        puissance_lampe = Spinbox(label_commandes, from_=0, to=1000, increment =0.1, textvariable=var2, command = mise_a_jour)
        var2.set(1.5)
        puissance_lampe.pack()

        Label(label_commandes, text=" ", fg="black").pack()

        Label(label_commandes, text="Taille Rendu : ", fg="black").pack()
        var3 = StringVar(label_commandes) 
        selec_taill = Spinbox(label_commandes, from_=2, to=1000, textvariable=var3, command = mise_a_jour)
        var3.set(TAILLE)
        selec_taill.pack()
        
        affichage()
        
    def Creer_fichier():
        global cadre, etat,frame_modification, filepath, depl_x, depl_y, depl_z, lamp_x, lamp_y, lamp_z, selec_taill , rota_x,rota_y,rota_z, i_objet, puissance_lampe, label_creation, type_objet,frame_creation

        if filepath == "#?":
            Enregistrer_fichier()
        try :
            open(filepath)
        except:
            word = ([0],[0],[])
            word = cube((0,0,0),30,word)
            word = array(word, dtype=object)
            ecrire_fichier2(filepath,word)
            
        etat = 2
        
        destructeur()
        i_objet = [0]

        Label(cadre, text="Créateur", fg="black", font=("Times",23,"bold"),background="yellow").pack()
        Label(cadre, text="Fichier : {}".format(filepath), fg="black").pack()
        Label(cadre, text="-----------------------", fg="black").pack()
        Label(cadre, text="Créateur : Emett Haddad ©".format(filepath), fg="black").pack(side = BOTTOM)
        Label(cadre, text="-----------------------", fg="black").pack(side = BOTTOM)

        #Visualisation
        
        label_commandes = LabelFrame(cadre, text="Visualisation", padx=20, pady=20)
        label_commandes.pack(side = RIGHT, expand="yes")
     
        Label(label_commandes).pack()
        Label(label_commandes, text="Rotation camera: (x,y,z)", fg="black").pack()
        var4 = StringVar(label_commandes) 
        rota_x = Spinbox(label_commandes, from_=0, to=360, textvariable=var4, command = mise_a_jour)
        var4.set(45)
        rota_x.pack()
        var5 = StringVar(label_commandes) 
        rota_y = Spinbox(label_commandes, from_=0, to=360, textvariable=var5, command = mise_a_jour)
        var5.set(45)
        rota_y.pack()
        var6 = StringVar(label_commandes) 
        rota_z = Spinbox(label_commandes, from_=0, to=360, textvariable=var6, command = mise_a_jour)
        var6.set(45)
        rota_z.pack()
        
        Label(label_commandes, text=" ", fg="black").pack()

        Label(label_commandes, text="Champ de vision: (x,y,z)", fg="black").pack()
        
        var = StringVar(label_commandes) 
        depl_x = Spinbox(label_commandes, from_=1, to=1000, textvariable=var, command = mise_a_jour)
        var.set(40)
        depl_x.pack()
        
        depl_y = Spinbox(label_commandes, from_=1, to=1000, command = mise_a_jour)
        depl_y.pack()
        depl_z = Spinbox(label_commandes, from_=100, to=1000, command = mise_a_jour)
        depl_z.pack()
        
        Label(label_commandes, text=" ", fg="black").pack()
        
        Label(label_commandes, text="Position lampe : (x,y,z)", fg="black").pack()
        lamp_x = Spinbox(label_commandes, from_=0, to=360, command = mise_a_jour)
        lamp_x.pack()
        lamp_y = Spinbox(label_commandes, from_=0, to=360, command = mise_a_jour)
        lamp_y.pack()
        lamp_z = Spinbox(label_commandes, from_=0, to=360, command = mise_a_jour)
        lamp_z.pack()

        Label(label_commandes, text=" ", fg="black").pack()
        
        Label(label_commandes, text="Puissance lampe :", fg="black").pack()
        var2 = StringVar(label_commandes)
        puissance_lampe = Spinbox(label_commandes, from_=0, to=1000, increment =0.1, textvariable=var2, command = mise_a_jour)
        var2.set(1.5)
        puissance_lampe.pack()

        Label(label_commandes, text=" ", fg="black").pack()

        Label(label_commandes, text="Taille Rendu : ", fg="black").pack()
        var3 = StringVar(label_commandes) 
        selec_taill = Spinbox(label_commandes, from_=2, to=1000, textvariable=var3, command = mise_a_jour)
        var3.set(TAILLE)
        selec_taill.pack()

        #Creation

        label_creation = LabelFrame(cadre, text="Creation", padx=20, pady=20)
        label_creation.pack(side = LEFT, expand="yes")
     
        Label(label_creation).pack()
        Label(label_creation, text="Type d'objet:", fg="black").pack()
        frame_creation = Frame(label_creation)
        frame_creation.pack()
        type_objet = Spinbox(label_creation,values=("Cube","Triangle","Modification") , command = mise_a_jour_liste)
        type_objet.pack()
        mise_a_jour_liste()

        bouton_suppr=Button(label_creation, text="Supprimer", command=supprimer_objet2, bg = "red")
        bouton_suppr.pack(side = BOTTOM)
        Label(label_creation, text=" ", fg="black").pack(side = BOTTOM)
        
        affichage()

    def mise_a_jour_liste():
        global type_objet, label_creation, frame_creation ,O_x, O_y , O_z, C_cube,B_x,B_y,B_z,A_x, A_y,A_z, C_x,C_y,C_z
        v_type_objet = str(type_objet.get())
        
        frame_creation.pack_forget()
        frame_creation = Frame(label_creation)
        frame_creation.pack(fill= "y")
        if v_type_objet == "Cube":
            Label(frame_creation, text="Cube (Origine x, y, z, et Cote) :", fg="black").pack()
            valx = StringVar(frame_creation) 
            O_x = Spinbox(frame_creation, from_=-100, to=100, textvariable=valx)
            valx.set(0)
            O_x.pack()
            valy = StringVar(frame_creation) 
            O_y = Spinbox(frame_creation, from_=-100, to=100, textvariable=valy)
            valy.set(0)
            O_y.pack()
            valz = StringVar(frame_creation) 
            O_z = Spinbox(frame_creation, from_=-100, to=100, textvariable=valz)
            O_z.pack()
            valz.set(0)
            
            Label(frame_creation, text=" ", fg="black").pack()
            C_cube = Spinbox(frame_creation, from_=1, to=100)
            C_cube.pack()
            
            Label(frame_creation, text=" ", fg="black").pack()
            bouton_aj=Button(frame_creation, text="Créer cube", command=ajout_cube, bg = "green")
            bouton_aj.pack()
        if v_type_objet == "Triangle":
            Label(frame_creation, text="Triangle 3 points (x,y,z) :", fg="black").pack()
            
            Label(frame_creation, text="A : ", fg="black").pack()
            valx = StringVar(frame_creation) 
            A_x = Spinbox(frame_creation, from_=-100, to=100, textvariable=valx)
            valx.set(0)
            A_x.pack()
            valy = StringVar(frame_creation) 
            A_y = Spinbox(frame_creation, from_=-100, to=100, textvariable=valy)
            valy.set(0)
            A_y.pack()
            valz = StringVar(frame_creation) 
            A_z = Spinbox(frame_creation, from_=-100, to=100, textvariable=valz)
            A_z.pack()
            valz.set(0)

            Label(frame_creation, text="B : ", fg="black").pack()
            valx2 = StringVar(frame_creation) 
            B_x = Spinbox(frame_creation, from_=-100, to=100, textvariable=valx2)
            valx2.set(0)
            B_x.pack()
            valy2 = StringVar(frame_creation) 
            B_y = Spinbox(frame_creation, from_=-100, to=100, textvariable=valy2)
            valy2.set(0)
            B_y.pack()
            valz2 = StringVar(frame_creation) 
            B_z = Spinbox(frame_creation, from_=-100, to=100, textvariable=valz2)
            B_z.pack()
            valz2.set(0)

            Label(frame_creation, text="C : ", fg="black").pack()
            valx3 = StringVar(frame_creation) 
            C_x = Spinbox(frame_creation, from_=-100, to=100, textvariable=valx3)
            valx3.set(0)
            C_x.pack()
            valy3 = StringVar(frame_creation) 
            C_y = Spinbox(frame_creation, from_=-100, to=100, textvariable=valy3)
            valy3.set(0)
            C_y.pack()
            valz3 = StringVar(frame_creation) 
            C_z = Spinbox(frame_creation, from_=-100, to=100, textvariable=valz3)
            C_z.pack()
            valz3.set(0)
            
            Label(frame_creation, text=" ", fg="black").pack()
            bouton_aj=Button(frame_creation, text="Créer triangle", command=ajout_triangle, bg = "green")
            bouton_aj.pack()
            
        if v_type_objet == "Modification":

            Origine,Rotation,Dilatation=(0,0,0),(0,0,0),(0,0,0)
            objet = lire_objet()
            
            if objet != 0:
                Dilatation,Rotation,Origine = objet[1]
            
            Label(frame_creation, text="Objet", fg="black").pack()
            Label(frame_creation, text="Origine (x, y, z) :", fg="black").pack()
            
            valx = StringVar(frame_creation) 
            A_x = Spinbox(frame_creation, from_=-100, to=100, textvariable=valx, increment =0.1)
            valx.set(Origine[0])
            A_x.pack()
            valy = StringVar(frame_creation) 
            A_y = Spinbox(frame_creation, from_=-100, to=100, textvariable=valy, increment =0.1)
            valy.set(Origine[1])
            A_y.pack()
            valz = StringVar(frame_creation) 
            A_z = Spinbox(frame_creation, from_=-100, to=100, textvariable=valz, increment =0.1)
            A_z.pack()
            valz.set(Origine[2])

            Label(frame_creation, text="Rotation (x, y, z) :", fg="black").pack()
            valx2 = StringVar(frame_creation) 
            B_x = Spinbox(frame_creation, from_=-100, to=100, textvariable=valx2, increment =0.1)
            valx2.set(Rotation[0])
            B_x.pack()
            valy2 = StringVar(frame_creation) 
            B_y = Spinbox(frame_creation, from_=-100, to=100, textvariable=valy2, increment =0.1)
            valy2.set(Rotation[1])
            B_y.pack()
            valz2 = StringVar(frame_creation) 
            B_z = Spinbox(frame_creation, from_=-100, to=100, textvariable=valz2, increment =0.1)
            B_z.pack()
            valz2.set(Rotation[2])

            Label(frame_creation, text="Dilatation (x, y, z) :", fg="black").pack()
            valx3 = StringVar(frame_creation) 
            C_x = Spinbox(frame_creation, from_=0, to=100, textvariable=valx3, increment =0.1)
            valx3.set(Dilatation[0])
            C_x.pack()
            valy3 = StringVar(frame_creation) 
            C_y = Spinbox(frame_creation, from_=0, to=100, textvariable=valy3, increment =0.1)
            valy3.set(Dilatation[1])
            C_y.pack()
            valz3 = StringVar(frame_creation) 
            C_z = Spinbox(frame_creation, from_=0, to=100, textvariable=valz3, increment =0.1)
            C_z.pack()
            valz3.set(Dilatation[2])
            
            Label(frame_creation, text=" ", fg="black").pack()
            bouton_mod=Button(frame_creation, text="Editer", command=modifier_objet, bg = "green")
            bouton_mod.pack()
            
    def ajout_cube():
        global filepath
        word = lire_fichier2(filepath)
        Origine = (int(O_x.get()), int(O_y.get()), int(O_z.get()))
        v_Cube = int(C_cube.get())
        word = cube(Origine,v_Cube,word) 
        word = array(word, dtype=object)
        ecrire_fichier2(filepath,word)
        affichage()
        
    def ajout_triangle():
        global filepath
        word = lire_fichier2(filepath)
        A = (int(A_x.get()), int(A_y.get()), int(A_z.get()))
        B = (int(B_x.get()), int(B_y.get()), int(B_z.get()))
        C = (int(C_x.get()), int(C_y.get()), int(C_z.get()))
        
        if not(A == B or B == C or A == C):       
            word = triangle(A,B,C,word)
            word = array(word, dtype=object)
            ecrire_fichier2(filepath,word)
            affichage()
        
    def lire_objet():
        global i_objet
        if etat == 2 and i_objet[0]!=0:
            word = lire_fichier2(filepath)
            i = i_objet[0]-1
            objet = word[2][i]
            return objet
        else:
            return 0
    def modifier_objet():
        global filepath
        word = lire_fichier2(filepath)
        objet = lire_objet()
        
        Origine = (float(A_x.get()), float(A_y.get()), float(A_z.get()))
        Rotation = (float(B_x.get()), float(B_y.get()), float(B_z.get()))
        Dilatation = (float(C_x.get()), float(C_y.get()), float(C_z.get()))
        
        objet = (objet[0],(Dilatation,Rotation,Origine))
        word [2][i_objet[0]-1] = objet
        
        word = array(word, dtype=object)
        ecrire_fichier2(filepath,word)
        
        affichage()
        
    def supprimer_objet(event):
        supprimer_objet2()
        
    def supprimer_objet2():
        global i_objet
        if etat == 2 and i_objet[0]!=0:
            word = lire_fichier2(filepath)
            
            i = i_objet[0]-1
            del word[2][i]
                
            word = array(word, dtype=object)
            ecrire_fichier2(filepath,word)
            i_objet = 0
            affichage()
        
    def affichage():
        global cadre_photo, filepath, rendu_objets,taille_image, etat

        taille_image = int(selec_taill.get())
        destructeur2()
        
        rota_camera = (int(rota_z.get()), int(rota_x.get()), int(rota_y.get()))
        depl_champ = (int(depl_x.get()), int(depl_y.get()), int(depl_z.get()))
        v_puissance_lampe = float(puissance_lampe.get())
        if depl_champ[1]==depl_champ[2]:
            depl_champ=(int(depl_x.get()), int(depl_y.get())+1, int(depl_z.get()))
        depl_lamp = (int(lamp_z.get()), int(lamp_x.get()), int(lamp_y.get()))

        rendu_objets = f_rendu(filepath,taille_image,rota_camera ,depl_champ, depl_lamp,i_objet,v_puissance_lampe)
        e_image = PhotoImage(file = "image_rendu.png")
        largeur = e_image.width()
        hauteur = e_image.height()
        canvas_image = Canvas(cadre_photo, width = largeur, height = hauteur)
        canvas_image.create_image(0,0, anchor = NW, image = e_image)
        canvas_image.pack()
        sleep(0.01)
        canvas_image.bind('<Button-1>',right_click)
        canvas_image.bind('<Button-2>',supprimer_objet)
        
        fenetre.mainloop()

    def Ouvrir_fichier():
        global filepath
        filepath = askopenfilename(title="Ouvrir un fichier 3d",initialdir=os.getcwd()+"\Objet 3d",filetypes=[('Fichier e3d','.e3d')])
        destructeur()
        Simulateur()

    def Enregistrer_fichier():
        global filepath
        filepath = asksaveasfilename(title="Enregistrer sous", initialdir=os.getcwd()+"\Objet 3d", filetypes = [('Fichier e3d','.e3d')])
        if filepath[-4:]!= ".e3d":
            filepath += ".e3d"
        Creer_fichier()
        
        
    def destructeur():
        global cadre
        cadre.pack_forget()
        cadre = Frame(fenetre, width=1500, height=1500, borderwidth=0)
        cadre.pack(fill= "y")
        
    def destructeur2():
        global cadre_photo
        cadre_photo.pack_forget()
        cadre_photo = Frame(cadre, width=1500, height=1500, borderwidth=0)
        cadre_photo.pack(fill= "y")
        
    def mise_a_jour():
        global etat
        if etat == 1:
            affichage()
        if etat == 2:
            affichage()
    def right_click(event):
        global i_objet, taille_image
        X = event.x
        Y = event.y

        if X < taille_image and Y < taille_image:
            i_objet = rendu_objets[Y,X]
        
        try:
            if str(type_objet.get()) == "Modification":
                mise_a_jour_liste()
        except:
            pass
        mise_a_jour()
      
    Interface_accueil()
    menubar = Menu(fenetre)
    menu = Menu(menubar, tearoff=0)
    menubar.add_command(label="Accueil",command=Interface_accueil)
    menubar.add_command(label="Simulateur",command=Simulateur)
    menubar.add_command(label="Créateur",command=Creer_fichier)
    menu1 = Menu(menubar, tearoff=0)
    menubar.add_cascade(label="Fichier", menu=menu1)
    menu1.add_command(label="Ouvrir un fichier",command=Ouvrir_fichier)
    menu1.add_command(label="Créer un fichier",command=Enregistrer_fichier)

    fenetre.config(menu=menubar)
    fenetre.mainloop()
main()
