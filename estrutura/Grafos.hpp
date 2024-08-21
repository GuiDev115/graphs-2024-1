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
#include <iostream>

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
    bool isDirecionado();

    // Verificar
    bool isConexo();
    bool isBipartido();
    bool isEuleriano();
    bool possuiCiclo();

    //Listar
    int componentesConexos();
    int componentesFortementeConexas();
    vector<int> trilhaEuleriana();
    vector<int> verticesArticulacao();
    vector<int> arestasPonte();

    // Gerar
    void arvoreDFS();
    void arvoreBFS();
    void arvoreGeradoraMinima();
    vector<int> ordemTopologica();
    int caminhoMinimo(int origem = 0, int destino = -1);
    int fluxoMaximo(int origem = 0, int destino = -1);
    vector<vector<int>> fechoTransitivo();

protected:
    void dfsComponentesFortes(int at, vector<int>& ids, vector<int>& low, vector<bool>& onStack, stack<int>& s, int& numeroComponentesFortes, int& id);
    void dfsTrilhaEuleriana(int v, vector<vector<int>>& adj, vector<int>& trilha);
    void dfsArticulacao(int at, int parent, vector<int>& ids, vector<int>& low, vector<bool>& visited, set<int>& articulations, int& id);
    void dfsArestasPonte(int at, int parent, vector<int>& ids, vector<int>& low, vector<bool>& visited, vector<int>& bridges, int& id);
    void dfsArvore(int v, vector<bool>& visitado, vector<pair<int, int>>& arestas, int& idAresta);
};

Grafo::Grafo(int V, string isDirected) {
    this->V = V;
    this->isDirected = (isDirected == "direcionado");
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

bool Grafo::isDirecionado() {
    return this->isDirected;
}

#include "algoritmos/Verificar.hpp"

#include "algoritmos/Listar.hpp"

#include "algoritmos/Gerar.hpp"

#endif
