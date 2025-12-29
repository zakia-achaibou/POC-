# Rapport — Planification de trajectoire d’un robot mobile par SLPSO (Self-Adaptive Learning Particle Swarm Optimization)

## Résumé
Ce rapport présente une preuve de concept (PoC) de planification de trajectoire pour robot mobile en environnement 2D avec obstacles statiques. Le problème est formulé comme une optimisation multi-objectif minimisant simultanément : (i) la longueur du chemin, (ii) le risque de collision, et (iii) la douceur (smoothness) de la trajectoire. La méthode étudiée est SLPSO, une variante de PSO intégrant quatre opérateurs d’apprentissage et un mécanisme auto-adaptatif des probabilités de sélection.

## 1) Introduction au problème et à sa modélisation

### 1.1 Problème de planification de trajectoire (Path Planning)

On cherche une trajectoire **sans collision** entre un point de départ **S** et une cible **T**, dans un environnement 2D contenant des obstacles statiques (formes diverses). Le problème est formulé comme une **optimisation sous contraintes** : minimiser une fonction objectif tout en garantissant l’absence de collision. 

### 1.2 Modélisation du chemin par waypoints dans un repère local

L’article propose de représenter un chemin par **D waypoints internes** (sans compter S et T).  
Les auteurs fixent les coordonnées **x’** des waypoints en divisant uniformément la distance \|ST\| en **D+1 segments** dans un repère local \(S - X'Y'\). L’optimisation ne cherche alors que les déviations **y’** sur des droites verticales \(L_1 \dots L_D\). 

Transformation local → global : on applique une rotation (angle \(\theta\) entre l’axe global et le segment ST) puis une translation par S. 

**Pourquoi cette modélisation ?**
- elle réduit la dimension utile : chaque particule de l’essaim n’encode que les **D valeurs y’** ;
- elle garantit que les points progressent dans la direction globale S→T (via x’ uniformes). 

---

## 2) Critères d’évaluation d’une solution

Les auteurs combinent trois critères (multi-objectif) via une somme pondérée (méthode additive). 

### 2.1 Longueur du chemin \(L(P)\)
Somme des distances euclidiennes entre points successifs (S, waypoints, T). 

### 2.2 Risque de collision \(R(X_{rob}, X_{obs})\)
Risque basé sur un **modèle Gaussien-like 2D** : le risque est non nul dans une zone d’influence autour des obstacles et nul au-delà, avec des paramètres de forme (variance/portée) choisis par l’auteur. 

> Dans une implémentation “preuve de concept”, on peut remplacer ce modèle continu par une version **échantillonnée le long des segments** (détection collision + pénalité de proximité), tout en conservant l’esprit de l’équation (5) (pénaliser la proximité).

### 2.3 Douceur \(S(P)\) (smoothness)
Somme des angles de déflexion (entre trois waypoints consécutifs). Plus le chemin “zigzague”, plus \(S(P)\) augmente. 

### 2.4 Fonction objectif globale
\[
J = w_1 L + w_2 R + w_3 S
\]
Les poids dépendent du contexte ; dans l’article : **\(w_1=0.6, w_2=0.3, w_3=0.1\)**. 

---

## 3) Dataset(s) de référence et métriques

### 3.1 Références “datasets” dans l’article

1) **Benchmark d’optimisation** : testbed CEC-2013 (28 fonctions) pour montrer que SLPSO est un bon optimiseur générique.   
2) **Environnement simulé** (ROS + Gazebo/Player-Stage) : workspace **12m × 12m**, obstacles variés, gap minimal ≈ 0.5m. Départ (1,1) → cible (11,11).   
3) **Environnement réel** (TurtleBot2) : workspace **12m × 12m**, gap minimal ≈ 0.6m, D=20.   

### 3.2 Métriques suivies
- \(L\) (m), \(R\) (degré de risque), \(S\) (rad) et coût global \(J\).   
- **Temps de calcul** / running time.   

### 3.3 Paramètres expérimentaux (article)
- Population **N=30** (PSO et SLPSO) ; **Itermax = 150** ; fréquence d’update des ratios **Uf = 3** ; \(\eta_3=1.496\).   
- D varie de **5 à 30** pour étudier l’impact du nombre de waypoints.   
- Critère d’arrêt : Itermax ou amélioration minimale (≈ 1% sur 10 itérations).   

---

## 4) Résolution du problème : du PSO standard à SLPSO

### 4.1 PSO standard (rappel)

Chaque particule i possède :
- une position \(X_i\) (ici : les D valeurs y’),
- une vitesse \(V_i\),
- un **Pbest** (meilleure position personnelle) et un **Gbest** (meilleure globale).   

La mise à jour classique combine inertie + apprentissage depuis Pbest et Gbest. 

### 4.2 SLPSO : idée clé

SLPSO emploie **4 opérateurs d’apprentissage** (a,b,c,d) :  
- **a (exploitation)** : apprendre depuis Pbest,  
- **b (convergence)** : apprendre depuis le meilleur voisin “le plus proche”,  
- **c (sortie d’optimum local)** : perturbation,  
- **d (exploration)** : apprendre depuis Gbest.   

Les probabilités de sélection sont **auto-adaptées** selon succès + progrès (fenêtre de taille Uf). 

---

## 5) Modélisation du “problem solving” (pseudocode, diagrammes) + complexité

### 5.1 Pseudocode (niveau notebook)

```text
Entrées : environnement, S, T, D, N, Itermax, Uf, paramètres (w1,w2,w3, η3, Vmax, bornes y')

1. Construire le repère local S-X'Y' ; fixer x'(d)=|ST|/(D+1)*d
2. Initialiser N particules (y' et V), Pbest=position initiale
3. Évaluer chaque particule → initialiser Gbest
4. Pour k = 1..Itermax :
      Pour chaque particule i :
          si k mod Uf == 0 : mettre à jour ratios (auto-adaptatif)
          choisir un opérateur (a,b,c,d) selon ratios
          mettre à jour vitesse/position selon opérateur
          gérer violations de bornes (Vmax + réflexion des positions)
          évaluer J ; mettre à jour Pbest/Gbest
5. Sortie : meilleur chemin = local_to_global(Gbest)
Ce flux correspond à l’Algorithm 1.

```
## 5.2 Complexité

Temps : O(D·T·N).

Mémoire : O(N·D).


## 6) Narratives et cas d’usage (reproductibles)
### 6.1 Cas d’usage minimal (POC) 
- Workspace 2D, obstacles statiques. 
- Départ S et cible T.
- Paramètres : \(N=30\), Itermax=150, D choisi (ex : 5).
### 6.2 Reproductibilité (seed)
python
import numpy as np, random
SEED = 42
np.random.seed(SEED)
random.seed(SEED)
 ## 7) Expériences numériques
 ## 8) Analyse :bénéfices, limites, hypothèses de simplification 
 ### 8.1 Points forts  
 SLPSO est supérieur à PSO/GA sur le coût global et la vitesse de convergence sur plusieurs D.
 ### 8.2 Limites + hypothèse de simplification 
 **Hypothèse de simplification proposée** : *ne traiter qu’un seul cas d’étude stable* (ex. **D=5**) et l’annoncer comme limite. 
 Justification : lorsque D augmente, la complexité et la difficulté augmentent ; l’article observe que les performances se dégradent au‑delà d’un certain point et que D≈20 est souvent un bon compromis. Autres limites POC : - risque approché (échantillonnage) ≠ risque exact (Eq.5), - lissage Ferguson spline non implémenté. 
 ### 8.3 Reproductibilité  
 Les paramètres clés sont fournis (N, Uf, η3, poids w1..w3), mais certains détails d’implémentation (collision/risk, discrétisation exacte) restent implicites. 
 
 ## 9) Plan de codage suivi 
 0. Modélisation de l’environnement
 1. Modélisation de la trajectoire du robot (TrajectoireModel)
 2. Évaluation du chemin (fonction de coût) (EvaluateurChemin)
 3. Implémentation du SLPSO ;
    3.1 Particle (création) ;
    3.2 4 stratégies (a,b,c,d) ;
    3.3 auto‑adaptation (ratios, Uf);
    3.4 gestion des bornes (Vmax + réflexion)
 4. Visualisation (trajectoire + convergence + métriques)
    
 ## 10) Références
 - Li G S, Chou W S. Path planning for mobile robot using self-adaptive learning particle swarm optimization. Sci China Inf Sci, 2018, 61(5): 052204, doi: 10.1007/s11432-016-911 .
