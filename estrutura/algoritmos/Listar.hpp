#ifndef LISTAR_HPP
#define LISTAR_HPP

#include "../Grafos.hpp"

// Função para listar componentes conexas em ordem lexicográfica
int Grafo::componentesConexos() {
    vector<bool> visitado(this->V, false);
    int numeroComponentes = 0;

    for (int i = 0; i < this->V; ++i) {
        if (!visitado[i]) {
            numeroComponentes++;
            stack<int> s;
            s.push(i);
            visitado[i] = true;

            while (!s.empty()) {
                int v = s.top();
                s.pop();

                for (int u : this->adj[v]) {
                    if (!visitado[u]) {
                        visitado[u] = true;
                        s.push(u);
                    }
                }
            }
        }
    }

    return numeroComponentes;
}

int Grafo::componentesFortementeConexas() {
    vector<int> ids(this->V, -1);
    vector<int> low(this->V, 0);
    vector<bool> onStack(this->V, false);
    stack<int> s;
    int id = 0;
    int numeroComponentesFortes = 0;

    for (int i = 0; i < this->V; ++i) {
        if (ids[i] == -1) {
            dfsComponentesFortes(i, ids, low, onStack, s, numeroComponentesFortes, id);
        }
    }

    return numeroComponentesFortes;
}

void Grafo::dfsComponentesFortes(int at, vector<int>& ids, vector<int>& low, vector<bool>& onStack, stack<int>& s, int& numeroComponentesFortes, int& id) {
    s.push(at);
    onStack[at] = true;
    ids[at] = low[at] = id++;

    for (int to : this->adj[at]) {
        if (ids[to] == -1) {
            dfsComponentesFortes(to, ids, low, onStack, s, numeroComponentesFortes, id);
            low[at] = min(low[at], low[to]);
        } else if (onStack[to]) {
            low[at] = min(low[at], ids[to]);
        }
    }

    if (ids[at] == low[at]) {
        while (true) {
            int node = s.top();
            s.pop();
            onStack[node] = false;
            if (node == at) break;
        }
        numeroComponentesFortes++;  // Incrementa o número de SCCs
    }
}

// Função auxiliar para DFS usada na trilha Euleriana
void Grafo::dfsTrilhaEuleriana(int v, vector<vector<int>>& adj, vector<int>& trilha) {
    while (!adj[v].empty()) {
        int u = adj[v].back();
        adj[v].pop_back();
        dfsTrilhaEuleriana(u, adj, trilha);
    }
    trilha.push_back(v);
}

// Função para encontrar uma trilha Euleriana priorizando a ordem lexicográfica dos vértices
vector<int> Grafo::trilhaEuleriana() {
    if (!isEuleriano()) return {};

    vector<int> trilha;
    vector<vector<int>> adj(this->V);

    // Copia as listas de adjacência e ordena para garantir ordem lexicográfica
    for (int u = 0; u < this->V; ++u) {
        adj[u] = this->adj[u];
        sort(adj[u].begin(), adj[u].end(), greater<int>()); // Ordem decrescente para usar com pop_back
    }

    int start = 0;
    for (int i = 0; i < this->V; ++i) {
        if (this->adj[i].size() % 2 == 1) {
            start = i;
            break;
        }
    }
    dfsTrilhaEuleriana(start, adj, trilha);

    reverse(trilha.begin(), trilha.end());
    return trilha;
}

// Função auxiliar para DFS usada em vértices de articulação
void Grafo::dfsArticulacao(int at, int parent, vector<int>& ids, vector<int>& low, vector<bool>& visited, set<int>& articulations, int& id) {
    visited[at] = true;
    ids[at] = low[at] = id++;
    int children = 0;

    for (int to : this->adj[at]) {
        if (to == parent) continue;
        if (!visited[to]) {
            dfsArticulacao(to, at, ids, low, visited, articulations, id);
            low[at] = min(low[at], low[to]);
            if (parent != -1 && low[to] >= ids[at]) {
                articulations.insert(at);
            }
            ++children;
        } else {
            low[at] = min(low[at], ids[to]);
        }
    }

    if (parent == -1 && children > 1) {
        articulations.insert(at);
    }
}

// Função para encontrar vértices de articulação
vector<int> Grafo::verticesArticulacao() {
    vector<int> low(this->V, 0), ids(this->V, -1);
    vector<bool> visited(this->V, false);
    set<int> articulations;
    int id = 0;

    for (int i = 0; i < this->V; ++i) {
        if (!visited[i]) {
            dfsArticulacao(i, -1, ids, low, visited, articulations, id);
        }
    }

    vector<int> result(articulations.begin(), articulations.end());
    sort(result.begin(), result.end());
    return result;
}

// Função auxiliar para DFS usada em arestas ponte
void Grafo::dfsArestasPonte(int at, int parent, vector<int>& ids, vector<int>& low, vector<bool>& visited, vector<int>& bridgeIDs, int& id) {
    visited[at] = true;
    ids[at] = low[at] = id++;

    for (int to : this->adj[at]) {
        if (to == parent) continue;
        if (!visited[to]) {
            dfsArestasPonte(to, at, ids, low, visited, bridgeIDs, id);
            low[at] = min(low[at], low[to]);
            if (low[to] > ids[at]) {
                bridgeIDs.push_back(this->pesos[at][to]); // Adiciona o ID da aresta à lista de pontes
            }
        } else {
            low[at] = min(low[at], ids[to]);
        }
    }
}

// Função para identificar arestas ponte
vector<int> Grafo::arestasPonte() {
    vector<int> low(this->V, 0), ids(this->V, -1);
    vector<bool> visited(this->V, false);
    vector<int> bridgeIDs; // Vetor para armazenar IDs das arestas ponte
    int id = 0;

    for (int i = 0; i < this->V; ++i) {
        if (!visited[i]) {
            dfsArestasPonte(i, -1, ids, low, visited, bridgeIDs, id);
        }
    }

    sort(bridgeIDs.begin(), bridgeIDs.end());
    return bridgeIDs;
}

#endif
