#ifndef Grafos_HPP
#define Grafos_HPP

#include <vector>
#include <string>
#include <limits.h>
#include <vector>
#include <stack>
#include <set>
#include <algorithm>
#include <queue>

using namespace std;

class Grafo {
private:
    int V; // Número de vértices
    bool isDirected; // Indica se o grafo é direcionado
    vector<vector<int>> adj; // Lista de adjacência
    vector<vector<int>> pesos; // Matriz de pesos (para grafos ponderados)
    
public:
    Grafo(int V, string isDirected);
    void addAresta(int u, int v, int w = 1);

    // Verificar
    bool isConexo();
    bool isBipartido();
    bool isEuleriano();
    bool possuiCiclo();

    //Listar
    vector<vector<int>> componentesConexas();
    vector<vector<int>> componentesFortementeConexas();
    vector<int> trilhaEuleriana();
    vector<int> verticesArticulacao();
    vector<pair<int, int>> arestasPonte();

protected:
    void dfsComponentesFortes(int at, vector<int>& ids, vector<int>& low, vector<bool>& onStack, stack<int>& s, vector<vector<int>>& sccs, int& id);
    void dfsTrilhaEuleriana(int v, vector<vector<int>>& adj, vector<int>& trilha);
    void dfsArticulacao(int at, int parent, vector<int>& ids, vector<int>& low, vector<bool>& visited, set<int>& articulations, int& id);
    void dfsArestasPonte(int at, int parent, vector<int>& ids, vector<int>& low, vector<bool>& visited, vector<pair<int, int>>& bridges, int& id);
};

Grafo::Grafo(int V, string isDirected) {
    this->V = V;
    this->isDirected = (isDirected == "direcionado") ? true : false;
    this->adj.resize(V);
    this->pesos.resize(V, vector<int>(V, INT_MAX));
}

void Grafo::addAresta(int u, int v, int w) {
    adj[u].push_back(v);
    pesos[u][v] = w;
    if (!isDirected) {
        adj[v].push_back(u);
        pesos[v][u] = w;
    }
}

#include "algoritmos/Verificar.hpp"

#include "algoritmos/Listar.hpp"

#endif
