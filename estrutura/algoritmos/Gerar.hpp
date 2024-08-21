#ifndef Gerar_HPP
#define Gerar_HPP

#include <iostream>
#include "../Grafos.hpp"

// Gerar Árvore de Profundidade (DFS Tree)
void Grafo::arvoreDFS() {
    vector<bool> visitado(V, false);
    vector<pair<int, int>> arestas;
    int idAresta = 0;

    dfsArvore(0, visitado, arestas, idAresta);

    // Imprime os identificadores das arestas na árvore DFS
    for (const auto& aresta : arestas) {
        cout << "Aresta ID: " << idAresta++ << " (" << aresta.first << ", " << aresta.second << ")" << endl;
    }
}

void Grafo::dfsArvore(int v, vector<bool>& visitado, vector<pair<int, int>>& arestas, int& idAresta) {
    visitado[v] = true;
    sort(adj[v].begin(), adj[v].end()); // Ordena os vértices adjacentes em ordem lexicográfica

    for (int u : adj[v]) {
        if (!visitado[u]) {
            arestas.push_back({v, u});
            dfsArvore(u, visitado, arestas, idAresta);
        }
    }
}

// Gerar Árvore de Largura (BFS Tree)
void Grafo::arvoreBFS() {
    vector<bool> visitado(V, false);
    queue<int> q;
    vector<pair<int, int>> arestas;
    int idAresta = 0;

    q.push(0);
    visitado[0] = true;

    while (!q.empty()) {
        int v = q.front();
        q.pop();

        sort(adj[v].begin(), adj[v].end()); // Ordena os vértices adjacentes em ordem lexicográfica

        for (int u : adj[v]) {
            if (!visitado[u]) {
                visitado[u] = true;
                arestas.push_back({v, u});
                q.push(u);
            }
        }
    }

    // Imprime os identificadores das arestas na árvore BFS
    for (const auto& aresta : arestas) {
        cout << "Aresta ID: " << idAresta++ << " (" << aresta.first << ", " << aresta.second << ")" << endl;
    }
}

// Gerar Árvore Geradora Mínima (MST)
void Grafo::arvoreGeradoraMinima() {
    if (isDirected) {
        cout << "Árvore Geradora Mínima não disponível para grafos direcionados." << endl;
        return;
    }

    vector<int> parent(V, -1);
    vector<int> chave(V, INT_MAX);
    vector<bool> inMST(V, false);
    vector<pair<int, int>> mstEdges;

    chave[0] = 0;
    parent[0] = -1;

    for (int count = 0; count < V - 1; ++count) {
        int u = -1;
        for (int i = 0; i < V; ++i) {
            if (!inMST[i] && (u == -1 || chave[i] < chave[u])) {
                u = i;
            }
        }

        inMST[u] = true;

        for (int v : adj[u]) {
            if (!inMST[v] && pesos[u][v] < chave[v]) {
                chave[v] = pesos[u][v];
                parent[v] = u;
            }
        }
    }

    for (int i = 1; i < V; ++i) {
        if (parent[i] != -1) {
            mstEdges.push_back({parent[i], i});
        }
    }

    sort(mstEdges.begin(), mstEdges.end());

    int idAresta = 0;
    for (const auto& aresta : mstEdges) {
        cout << "Aresta ID: " << idAresta++ << " (" << aresta.first << ", " << aresta.second << ")" << endl;
    }
}

// Ordem Topológica
vector<int> Grafo::ordemTopologica() {
    if (!isDirected) {
        cout << "Ordem Topológica não disponível para grafos não direcionados." << endl;
        return {};
    }

    vector<int> inDegree(V, 0);
    vector<int> ordem;
    priority_queue<int, vector<int>, greater<int>> q;

    for (int u = 0; u < V; ++u) {
        for (int v : adj[u]) {
            inDegree[v]++;
        }
    }

    for (int i = 0; i < V; ++i) {
        if (inDegree[i] == 0) {
            q.push(i);
        }
    }

    while (!q.empty()) {
        int u = q.top();
        q.pop();
        ordem.push_back(u);

        for (int v : adj[u]) {
            if (--inDegree[v] == 0) {
                q.push(v);
            }
        }
    }

    if (ordem.size() != static_cast<size_t>(V)) {
        cout << "O grafo tem um ciclo, ordem topológica não é possível." << endl;
        return {};
    }

    return ordem;
}

// Caminho Mínimo entre dois vértices (Dijkstra)
int Grafo::caminhoMinimo(int origem, int destino) {
    if (destino == -1) destino = V - 1;

    vector<int> dist(V, INT_MAX);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    dist[origem] = 0;
    pq.push({0, origem});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (int v : adj[u]) {
            int peso = pesos[u][v];
            if (dist[u] + peso < dist[v]) {
                dist[v] = dist[u] + peso;
                pq.push({dist[v], v});
            }
        }
    }

    return dist[destino];
}

// Fluxo Máximo (Edmonds-Karp)
int Grafo::fluxoMaximo(int origem, int destino) {
    if (!isDirected) {
        cout << "Fluxo Máximo não disponível para grafos não direcionados." << endl;
        return 0;
    }
    if (destino == -1) destino = V - 1;

    vector<vector<int>> capacidade(V, vector<int>(V, 0));
    for (int u = 0; u < V; ++u) {
        for (int v : adj[u]) {
            capacidade[u][v] = pesos[u][v];
        }
    }

    int fluxoTotal = 0;
    vector<int> parent(V);

    auto bfs = [&](int s, int t) -> bool {
        fill(parent.begin(), parent.end(), -1);
        queue<pair<int, int>> q;
        q.push({s, INT_MAX});

        while (!q.empty()) {
            int u = q.front().first;
            int fluxo = q.front().second;
            q.pop();

            for (int v : adj[u]) {
                if (parent[v] == -1 && capacidade[u][v]) {
                    parent[v] = u;
                    int novo_fluxo = min(fluxo, capacidade[u][v]);
                    if (v == t) return true;
                    q.push({v, novo_fluxo});
                }
            }
        }

        return false;
    };

    while (bfs(origem, destino)) {
        int fluxo = INT_MAX;
        for (int u = destino; u != origem; u = parent[u]) {
            int p = parent[u];
            fluxo = min(fluxo, capacidade[p][u]);
        }

        for (int u = destino; u != origem; u = parent[u]) {
            int p = parent[u];
            capacidade[p][u] -= fluxo;
            capacidade[u][p] += fluxo;
        }

        fluxoTotal += fluxo;
    }

    return fluxoTotal;
}

// Fecho Transitivo (Algoritmo de Warshall)
vector<vector<int>> Grafo::fechoTransitivo() {
    if (!isDirected) {
        cout << "Fecho Transitivo não disponível para grafos não direcionados." << endl;
        return {};
    }

    vector<vector<int>> fecho(V, vector<int>(V, 0));

    for (int i = 0; i < V; ++i) {
        for (int j : adj[i]) {
            fecho[i][j] = 1;
        }
    }

    for (int k = 0; k < V; ++k) {
        for (int i = 0; i < V; ++i) {
            for (int j = 0; j < V; ++j) {
                fecho[i][j] = fecho[i][j] || (fecho[i][k] && fecho[k][j]);
            }
        }
    }

    return fecho;
}

#endif