from Bibliotheque_fonctions_3d_et_2d import *


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
def main():
    word = ([0],[0],[])
    word = cube((0,0,0),10,word)
    word = cube((30,10,0),5,word)
    word = cube((10,30,0),5,word)
    word = cube((10,10,10),8,word)
    word = carre((-10,-10,-10),30,word)
    word = triangle((40,40,40),(10,20,20),(50,40,20),word)
    word = array(word, dtype = object)
    n = 2
    ecrire_fichier("Word n°{}.e3d".format(n),word)
    return 0

if __name__ == '__main__':
    main()
