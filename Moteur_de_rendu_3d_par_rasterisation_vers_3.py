# Version 3
from Bibliotheque_fonctions_3d_et_2d import *

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
