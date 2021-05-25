from cx_Freeze import setup, Executable
# On appelle la fonction setup
nom="Interface_graphique"
setup(
    name = nom,
    version = "1",
    description = "Interface_graphique",
    executables = [Executable(nom+".py")],
)

