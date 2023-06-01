Travail réalisé dans le cadre de l'enseignement d'intégration de la séquence
thématique 4 "Internet des objets".

## Objectif

L'objectif est de diriger un robot en modifiant son environnement. En fonction
de la personne qui se tient devant la caméra embarquée, le robot effectue
différentes actions.

## Utilisation

L'ensemble des machines participantes doivent être sur le même réseau.

Pour que les *peers* se reconnaissent, il faut qu'un routeur soit présent sur
ce réseau. Pour stocker des informations relatives à la reconnaissance faciale,
il faut le doter de capacités de stockage :

```
zenohd --cfg='plugins/storage_manager/storages/demo:{key_expr:"elrobot/vectors/**",volume:"memory"}'
```

Le robot doit exécuter les scripts présents dans `robot`. Respectivement, ils
envoient la capture vidéo sur le réseau via [zenoh](https://zenoh.io), et
effectuent les mouvements commandés sur le réseau.

La reconnaissance faciale opère en deux phases : la détection de têtes dans
l'image, puis l'idenfication de la personne. Il y a deux possibilités : ou bien
ces phases sont effectuées sur deux machines différentes (`detect.py` et
`recognise.py`), ou bien elles sont effectuées sur la même machine
(`command_robot.py`). Le choix doit être effectué en fonction du coût
d'utilisation du réseau et des performances des machines.
