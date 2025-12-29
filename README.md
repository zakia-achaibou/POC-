# SLPSO Path Planning — Proof of Concept (PoC)

Implémentation **preuve de concept** de la planification de trajectoire d’un robot mobile en environnement 2D avec obstacles statiques, en comparant **SLPSO** (Self-Adaptive Learning PSO) avec deux baselines : **PSO standard** et **GA** (Algorithme Génétique).

Le pipeline suit l’idée de l’article : représenter un chemin par **D waypoints** (déviations \(y'\) dans un repère local \(S-X'Y'\)), puis optimiser une fonction objectif multi-objectif (longueur, risque, douceur).

---

## Objectif

- Générer une trajectoire entre **S (start)** et **T (target)** évitant les obstacles.
- Minimiser une fonction de coût :

\[
J = w_1 L + w_2 R + w_3 S
\]

où :
- \(L\) = longueur du chemin  
- \(R\) = risque de collision / proximité obstacles  
- \(S\) = douceur (smoothness) / pénalisation des virages brusques  

---

## Structure du projet

### 0) Modélisation de l’environnement
- Obstacles statiques : cercle / rectangle / triangle
- Méthode attendue :
  - `mon_env.obstacles` (liste de dicts)
  - `mon_env.est_en_collision(point)` (collision point-obstacle)

### 1) Modélisation de la trajectoire (`TrajectoireModel`)
- Construction du repère local \(S-X'Y'\)
- Fixation des \(x'\) uniformes
- Transformation `local_to_global(y_primes)` pour obtenir \((x,y)\)

### 2) Évaluation du chemin (`EvaluateurChemin`)
- `calculer_longueur(points)`
- `calculer_risque(points)`  
  - PoC : risque par **échantillonnage** le long des segments + pénalité collision
- `calculer_douceur(points)`
- `evaluer(points)` : retourne \(J\)

### 3) Optimisation
- `SLPSO` : implémentation de l’article (4 opérateurs + auto-adaptation)
- `PSO standard` : baseline classique (Pbest + Gbest)
- `GA` : baseline simple (tournament + crossover + mutation)

### 4) Visualisation
- Trajectoire optimale sur la carte
- Courbe de convergence \(J\) par itération/génération
- Graphe comparatif : **SLPSO vs PSO vs GA**

---

## Installation

Pré-requis Python :
- `numpy`
- `matplotlib`
- (optionnel) `pandas`

Installation rapide :
```bash
pip install numpy matplotlib pandas
