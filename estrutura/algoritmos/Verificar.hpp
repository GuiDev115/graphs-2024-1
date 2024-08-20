#ifndef Verificar_HPP
#define Verificar_HPP

#include "../Grafos.hpp"

// Função para verificar se o grafo é conexo (conectividade fraca)
bool Grafo::isConexo() {
    vector<bool> visitado(V, false);
    queue<int> q;
    int componentesConexas = 0;

    for (int i = 0; i < V; ++i) {
        if (!visitado[i]) {
            componentesConexas++;
            if (componentesConexas > 1) return false;

            q.push(i);
            visitado[i] = true;

            while (!q.empty()) {
                int u = q.front();
                q.pop();

                for (int v : adj[u]) {
                    if (!visitado[v]) {
                        visitado[v] = true;
                        q.push(v);
                    }
                }
            }
        }
    }
    return true;
}

// Função para verificar se o grafo é bipartido
bool Grafo::isBipartido() {
    vector<int> cores(V, -1);
    queue<int> q;

    for (int i = 0; i < V; ++i) {
        if (cores[i] == -1) {
            cores[i] = 0;
            q.push(i);

            while (!q.empty()) {
                int u = q.front();
                q.pop();

                for (int v : adj[u]) {
                    if (cores[v] == -1) {
                        cores[v] = 1 - cores[u];
                        q.push(v);
                    } else if (cores[v] == cores[u]) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

// Função para verificar se o grafo é Euleriano
bool Grafo::isEuleriano() {
    if (!isConexo()) return false;

    int verticesImpares = 0;
    for (int i = 0; i < V; ++i) {
        if (adj[i].size() % 2 != 0) {
            verticesImpares++;
        }
    }
    if (isDirected) {
        return verticesImpares == 0; // Condição para ciclo euleriano em grafos direcionados
    } else {
        return verticesImpares == 0 || verticesImpares == 2; // Para grafos não direcionados
    }
}

// Função para verificar se o grafo possui ciclo
bool Grafo::possuiCiclo() {
    vector<bool> visitado(V, false);
    vector<int> parent(V, -1);

    for (int i = 0; i < V; ++i) {
        if (!visitado[i]) {
            // Início da DFS para detectar ciclos
            stack<int> s;
            s.push(i);
            visitado[i] = true;

            while (!s.empty()) {
                int v = s.top();
                s.pop();

                for (int u : adj[v]) {
                    if (!visitado[u]) {
                        parent[u] = v;
                        s.push(u);
                        visitado[u] = true;
                    } else if (u != parent[v]) {
                        return true; // Ciclo detectado
                    }
                }
            }
        }
    }
    return false; // Nenhum ciclo encontrado
}

#endif
