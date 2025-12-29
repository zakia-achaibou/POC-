Rapport — Planification de trajectoire d’un robot mobile par SLPSO (Self-Adaptive Learning Particle Swarm Optimization)

Résumé  
Ce rapport présente une preuve de concept (PoC) de planification de trajectoire pour robot mobile en environnement 2D avec obstacles statiques. Le problème est formulé comme une optimisation multi-objectif minimisant simultanément : (i) la longueur du chemin, (ii) le risque de collision, et (iii) la douceur (smoothness) de la trajectoire. La méthode étudiée est SLPSO, une variante de PSO intégrant quatre opérateurs d’apprentissage et un mécanisme auto-adaptatif des probabilités de sélection.

## 1) Introduction au problème et à sa modélisation

### 1.1 Problème de planification de trajectoire (Path Planning)
On cherche une trajectoire sans collision entre un point de départ S et une cible T, dans un environnement 2D contenant des obstacles statiques de formes diverses. Le problème est formulé comme une optimisation sous contraintes : minimiser une fonction objectif tout en garantissant l’absence de collision.

### 1.2 Modélisation du chemin par waypoints dans un repère local
L’article propose de représenter un chemin par D waypoints internes (sans compter S et T). Les auteurs fixent les coordonnées x’ des waypoints en divisant uniformément la distance $\|ST\|$ en D+1 segments dans un repère local $(S - X'Y')$. L’optimisation ne cherche alors que les déviations y’ sur des droites verticales $L_1 \dots L_D$.

Transformation local → global : on applique une rotation d'angle $\theta$ (entre l’axe global et le segment ST) puis une translation par S.

Avantages de cette modélisation :  
Elle réduit la dimension utile : chaque particule n’encode que les D valeurs y’.  
Elle garantit que les points progressent dans la direction globale S→T.

## 2) Critères d’évaluation d’une solution
Les auteurs combinent trois critères via une somme pondérée (méthode additive).

### 2.1 Longueur du chemin $L(P)$
Somme des distances euclidiennes entre points successifs (S, waypoints, T).

### 2.2 Risque de collision $R(X_{rob}, X_{obs})$
Risque basé sur un modèle Gaussien-like 2D : le risque est non nul dans une zone d’influence autour des obstacles et nul au-delà.

Note PoC : Pour la preuve de concept, ce modèle peut être remplacé par une version échantillonnée le long des segments (détection collision + pénalité de proximité).

### 2.3 Douceur $S(P)$ (smoothness)
Somme des angles de déflexion entre trois waypoints consécutifs. Plus le chemin zigzague, plus $S(P)$ augmente.

### 2.4 Fonction objectif globale
$$J = w_1 L + w_2 R + w_3 S$$  
Poids utilisés dans l’article : $w_1=0.6, w_2=0.3, w_3=0.1$.

## 3) Datasets de référence et métriques

### 3.1 Références dans l’article
Benchmark d’optimisation : CEC-2013 (28 fonctions) pour valider SLPSO comme optimiseur générique.  
Environnement simulé (ROS/Gazebo) : Workspace 12m × 12m, obstacles variés, gap minimal ≈ 0.5m. S(1,1) → T(11,11).  
Environnement réel (TurtleBot2) : Workspace 12m × 12m, gap minimal ≈ 0.6m, D=20.

### 3.2 Métriques suivies
$L$ (m), $R$ (degré de risque), $S$ (rad) et coût global $J$.  
Temps de calcul (running time).

### 3.3 Paramètres expérimentaux
Population N=30.  
Itermax = 150.  
Fréquence d’update des ratios Uf = 3.  
Paramètres de mouvement : $\eta_3=1.496$ et inertie $w=0.73$.

## 4) Résolution du problème : de PSO à SLPSO

### 4.1 PSO standard
Chaque particule possède une position $X_i$ (les $D$ valeurs $y'$), une vitesse $V_i$, un Pbest (meilleur personnel) et un Gbest (meilleure globale). La mise à jour classique combine inertie et apprentissage.

### 4.2 SLPSO : idée clé
SLPSO emploie 4 opérateurs d’apprentissage (a, b, c, d) :  
a (exploitation) : apprentissage depuis Pbest.  
b (convergence) : apprentissage depuis le meilleur voisin proche.  
c (sortie d’optimum local) : perturbation.  
d (exploration) : apprentissage depuis Gbest.

Probabilités de sélection auto-adaptées selon le succès et le progrès sur une fenêtre $Uf$.

## 5) Modélisation du “problem solving”

### 5.1 Pseudocode
Construire le repère local S-X'Y' ; fixer $x'(d)$.  
Initialiser $N$ particules ($y'$ et $V$), Pbest = position initiale.  
Évaluer chaque particule → initialiser Gbest.  
Pour $k = 1 \dots Itermax$ :  
Si $k \pmod{Uf} == 0$ : mettre à jour les ratios d'opérateurs.  
Choisir un opérateur (a, b, c, d) selon les ratios.  
Mettre à jour vitesse/position ; gérer les violations de bornes ($Vmax$ + réflexion).  
Évaluer $J$ ; mettre à jour Pbest/Gbest.  
Sortie : meilleur chemin via local_to_global(Gbest).

### 5.2 Complexité
Temps : $O(D \cdot T \cdot N)$.  
Mémoire : $O(N \cdot D)$.

Détail de la complexité en temps (PoC)  
Le coût dominant est le calcul du risque : $O(T \cdot N \cdot D \cdot n\_samples \cdot M)$, où $M$ est le nombre d'obstacles et $n\_samples$ le nombre de points d’échantillonnage par segment.

## 6) Cas d’usage et reproductibilité
POC : Workspace 2D, obstacles statiques, $N=30, Itermax=150, D=5$.  
Reproductibilité : Utilisation de SEED = 42 pour stabiliser les résultats aléatoires.

## 7) Analyse : bénéfices et limites

### 7.1 Points forts
SLPSO est statistiquement supérieur à PSO et GA sur le coût global et la vitesse de convergence.

### 7.2 Limites et simplifications (POC)
Hypothèse : Utilisation de $D=5$ pour la stabilité, bien que l'article recommande $D=20$ comme compromis optimal.  
Simplifications : Risque approché par échantillonnage et absence de lissage par spline de Ferguson.

## 8) Références
Li G.S. et al., Path planning for mobile robot using self-adaptive learning particle swarm optimization. Sci China Inf Sci, 2018, 61(5): 052204.
