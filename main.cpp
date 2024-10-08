/*
Trabalho de Grafos
    Guilherme Fabricio Brito da Rosa - 202120497
    Julio Henrique Oliveira Mariano - 202110829
*/

#include <iostream>
#include <sstream>
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
    Grafo(int V, bool isDirected);
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
    int arestasPonte();

    // Gerar
    vector<int> arvoreDFS();
    vector<int> arvoreBFS(int v);
    void arvoreGeradoraMinima();
    vector<int> ordemTopologica();
    int caminhoMinimo(int origem = 0, int destino = -1);
    int fluxoMaximo(int origem = 0, int destino = -1);
    vector<int> fechoTransitivo(int vertice);

// auxiliares
protected:
    void dfsConexo(int v, vector<bool> &visitado);
    Grafo transpor();
    void dfsComponentesFortes(int at, vector<int>& ids, vector<int>& low, vector<bool>& onStack, stack<int>& s, int& numeroComponentesFortes, int& id);
    void dfsTrilhaEuleriana(int v, vector<vector<int>>& adj, vector<int>& trilha);
    void dfsArticulacao(int at, int parent, vector<int>& ids, vector<int>& low, vector<bool>& visited, set<int>& articulations, int& id);
    void dfsArestasPonte(int at, int parent, vector<int>& ids, vector<int>& low, vector<bool>& visited, vector<int>& bridges, int& id);
    void dfsArvore(int v, vector<bool>& visitado, vector<int>& arestas);
};

Grafo::Grafo(int V, bool isDirected) {
    this->V = V;
    this->isDirected = isDirected;
    this->adj.resize(V);
    this->pesos.resize(V, vector<int>(V, INT_MAX));
}

// Adiciona uma aresta ao grafo
void Grafo::addAresta(int u, int v, int w) {
    adj[u].push_back(v);
    pesos[u][v] = w;
    if (!isDirected) {
        adj[v].push_back(u);
        pesos[v][u] = w;
    }
}
// Verificar se o grafo é direcionado
bool Grafo::isDirecionado() {
    return this->isDirected;
}
//Fecho Transitivo 
    vector<int> Grafo::fechoTransitivo(int vertice) {
        vector<int> fecho;
        vector<bool> visitado(V, false);
        queue<int> fila;

    fila.push(vertice);
    visitado[vertice] = true;

    while (!fila.empty()) {
        int v = fila.front();
        fila.pop();
        fecho.push_back(v);

        for (int adj : adj[v]) {
            if (!visitado[adj]) {
                visitado[adj] = true;
                fila.push(adj);
            }
        }
    }
    return fecho;
}

//Verificar
// Função para verificar se o grafo é conexo (conectividade fraca)
bool Grafo::isConexo() {
    if (isDirected) {
        // Verifica a conectividade do grafo original
        vector<bool> visitado(V, false);
        dfsConexo(0, visitado);
        if (find(visitado.begin(), visitado.end(), false) != visitado.end()) {
            return false;
        }

        // Cria o grafo transposto
        Grafo grafoTransposto = this->transpor();

        // Verifica a conectividade do grafo transposto
        visitado.assign(V, false);
        grafoTransposto.dfsConexo(0, visitado);
        return find(visitado.begin(), visitado.end(), false) == visitado.end();
    } else {
        vector<bool> visitado(V, false);
        dfsConexo(0, visitado);
        return find(visitado.begin(), visitado.end(), false) == visitado.end();
    }
}

// Função auxiliar para DFS usada em grafos conexos
void Grafo::dfsConexo(int v, vector<bool> &visitado) {
    visitado[v] = true;

    for (int u : adj[v]) {
        if (!visitado[u]) {
            dfsConexo(u, visitado);
        }
    }
}

// Função para transpor o grafo
Grafo Grafo::transpor() {
    Grafo grafoTransposto(V, isDirected);

    for (int v = 0; v < V; ++v) {
        for (int u : adj[v]) {
            grafoTransposto.adj[u].push_back(v);
        }
    }
    return grafoTransposto;
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
    if (!isConexo()) {
        return false;
    }

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


//Listar
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

            while (!s.empty()) { // DFS
                int v = s.top();
                s.pop();

                for (int u : this->adj[v]) { // Vizinhos de v
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

// Função para listar componentes fortemente conexas
int Grafo::componentesFortementeConexas() {
    vector<int> ids(this->V, -1);
    vector<int> low(this->V, 0);
    vector<bool> onStack(this->V, false);
    stack<int> s;
    int id = 0;
    int numeroComponentesFortes = 0;

    for (int i = 0; i < this->V; ++i) {
        if (ids[i] == -1) {
            dfsComponentesFortes(i, ids, low, onStack, s, numeroComponentesFortes, id); // DFS
        }
    }

    return numeroComponentesFortes;
}

// Função auxiliar para DFS usada em componentes fortemente conexas
void Grafo::dfsComponentesFortes(int at, vector<int>& ids, vector<int>& low, vector<bool>& onStack, stack<int>& s, int& numeroComponentesFortes, int& id) {
    s.push(at);
    onStack[at] = true;
    ids[at] = low[at] = id++;

    for (int to : this->adj[at]) { // Vizinhos de at
        if (ids[to] == -1) { // Se to não foi visitado
            dfsComponentesFortes(to, ids, low, onStack, s, numeroComponentesFortes, id);
            low[at] = min(low[at], low[to]);
        } else if (onStack[to]) { // Se to está na pilha
            low[at] = min(low[at], ids[to]);
        }
    }

    if (ids[at] == low[at]) { // Se at é a raiz de uma SCC
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
    while (!adj[v].empty()) { // Enquanto houverem vizinhos
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
    for (int i = 0; i < this->V; ++i) { // Encontra o vértice de grau ímpar
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

    for (int to : this->adj[at]) { // Vizinhos de at
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
int Grafo::arestasPonte() {
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
    return bridgeIDs.size();
}

//Gerar
// Gerar Árvore de Profundidade (DFS Tree)
vector<int> Grafo::arvoreDFS() {
    vector<bool> visitado(V, false);
    vector<int> arestas;
    int idAresta = 0;

    dfsArvore(0, visitado, arestas);

    // Verifica se há vértices desconectados
    bool grafoDesconexo = false;
    for (bool v : visitado) { // Se algum vértice não foi visitado
        if (!v) {
            grafoDesconexo = true;
            break;
        }
    }

    return arestas;
}

// Função auxiliar para DFS usada em árvores
void Grafo::dfsArvore(int v, vector<bool>& visitado, vector<int>& arestas) {
    visitado[v] = true;

    // Ordena os vizinhos do vértice v para garantir a ordem lexicográfica
    vector<pair<int, int>> vizinhos;
    for (int i = 0; i < adj[v].size(); ++i) {
        int u = adj[v][i];
        vizinhos.push_back({u, i}); // Armazena o vértice adjacente e sua posição (ID da aresta)
    }
    sort(vizinhos.begin(), vizinhos.end());

    for (auto& vizinho : vizinhos) {
        int u = vizinho.first;
        int idAresta = vizinho.second;
        if (!visitado[u]) {
            arestas.push_back(idAresta); // Adiciona o ID da aresta (posição na lista) à lista
            dfsArvore(u, visitado, arestas);
        }
    }
}

// Gerar Árvore de Largura (BFS Tree)
vector<int> Grafo::arvoreBFS(int v = 0) {
    vector<bool> visitado(V, false);
    vector<int> arestas;
    queue<int> q;

    q.push(v);
    visitado[v] = true;

    while (!q.empty()) {
        int i = q.front();
        q.pop();

        // Ordena os vizinhos do vértice i para garantir a ordem lexicográfica
        vector<pair<int, int>> vizinhos;
        for (int j = 0; j < adj[i].size(); ++j) {
            int u = adj[i][j];
            vizinhos.push_back({u, j}); // Armazena o vértice adjacente e sua posição (ID da aresta)
        }
        sort(vizinhos.begin(), vizinhos.end());

        for (auto& vizinho : vizinhos) {
            int u = vizinho.first;
            int idAresta = vizinho.second;
            if (!visitado[u]) {
                visitado[u] = true;
                q.push(u);
                arestas.push_back(idAresta); // Adiciona o ID da aresta (posição na lista) à lista
            }
        }
    }

    return arestas;
}

// Gerar Árvore Geradora Mínima (MST)
void Grafo::arvoreGeradoraMinima() {
    if (isDirected) {
        cout << "Árvore Geradora Mínima não disponível para grafos direcionados." << endl;
        return;
    }

// Algoritmo de Prim
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
// Imprime as arestas da MST
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

// Gerar Ordem Topológica
vector<int> Grafo::ordemTopologica() {
    if (!isDirected) {
        cout << "Ordem Topológica não disponível para grafos não direcionados." << endl;
        return {};
    }

    vector<int> inDegree(V, 0);
    vector<int> ordem;
    priority_queue<int, vector<int>, greater<int>> q;

    for (int u = 0; u < V; ++u) { // Calcula o grau de entrada de cada vértice
        for (int v : adj[u]) {
            inDegree[v]++;
        }
    }

    for (int i = 0; i < V; ++i) { // Adiciona os vértices com grau de entrada 0 à fila
        if (inDegree[i] == 0) {
            q.push(i);
        }
    }

    while (!q.empty()) { // Processa os vértices em ordem topológica
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

    auto bfs = [&](int s, int t) -> bool { // Encontra o caminho de s a t
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

    while (bfs(origem, destino)) { // Encontra o caminho de origem a destino
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

int main() {
    string linha;
    getline(cin, linha);

    istringstream iss(linha);

    int numVertices, numAresta;

    string direcionado_ou_nao_direcionado;

    // Leitura do número de vértices e arestas
    cin >> numVertices >> numAresta;

    // Leitura do tipo do grafo (direcionado ou não direcionado)
    cin >> direcionado_ou_nao_direcionado;

    // Inicializa o grafo
    Grafo g(numVertices, (direcionado_ou_nao_direcionado == "direcionado"));

    // Leitura das arestas
    for (int i = 0; i < numAresta; ++i) {
        int arestaID, u, v, w;
        cin >> arestaID >> u >> v >> w;
        g.addAresta(u, v, w);
    }

    int opcao;
    while (iss >> opcao) { // Leitura das opções
        switch (opcao) {
            case 0: //conexo
                cout << g.isConexo() << endl;
                break;
            case 1: //bipartido
                cout << g.isBipartido() << endl;
                break;
            case 2: // Euleriano
                cout << g.isEuleriano() << endl;
                break;
            case 3: //Ciclo
                cout << g.possuiCiclo() << endl;
                break;
            case 4: //conexas
                if(g.isDirecionado()){
                    cout << -1 << endl;
                }else{
                    cout << g.componentesConexos() << endl;
                }
                break;
            case 5: // fortemente
                if(!g.isDirecionado()){
                    cout << -1 << endl;
                }else{
                    cout << g.componentesFortementeConexas() << endl;
                }
                break;
            case 6: // somente unicos
                if (g.isDirecionado()) {
                    cout << -1 << endl;
                } else {
                    vector<int> verticesArticu = g.verticesArticulacao();
                    if(verticesArticu.size() > 0)
                        for (int v : g.verticesArticulacao()) {
                            cout << v << " ";
                        }
                    else{
                        cout << 0;
                    }
                    cout << endl;
                }
                break;
            case 7: // calcular quantas arestas pontes possui 
                if (g.isDirecionado()) {
                    cout << -1 << endl;
                } else {
                    cout << g.arestasPonte() << endl;
                }
                break;
            case 8: // Imprimir a avore em profundidade
                for (int id : g.arvoreDFS()) {
                    cout<< id << " ";
                }
                cout<<endl;
                break;
            case 9: // Imprimir a arvore de largura
                for (int id : g.arvoreBFS()) {
                    cout<< id << " ";
                }
                break;
            case 11: {
                g.arvoreBFS();
                break;
            }
            case 13: {
                if (g.isDirecionado()) {
                    vector<int> ordem = g.ordemTopologica();
                    for (int v : ordem) {
                            cout << v << " ";
                    }
                        cout << endl;
                } else {
                        cout << -1 <<  endl;
                }
                break;
            }
            case 12: {
                int origem = 0, destino = numVertices - 1;
                int caminho = g.caminhoMinimo(origem, destino);
                    cout << caminho <<  endl;
                break;
            }
            case 15: {
                if (g.isDirecionado()) {
                    int origem = 0, destino = numVertices - 1;
                    int fluxo = g.fluxoMaximo(origem, destino);
                        cout << fluxo <<  endl;
                } else {
                        cout << -1 <<  endl;
                }
                break;
            }
            case 14: { //fecho Transitivo

                if (g.isDirecionado()) {
                    int vertice = 0;
                        vector<int> fecho = g.fechoTransitivo(vertice);
                        sort(fecho.begin(), fecho.end()); // Ordenar lexicograficamente
                    for (int v : fecho) {
                        cout << v << " ";
                    }
                    cout <<     endl;
                } else {
                        cout << "-1" <<     endl;
                }
                break; 
            }
            // case 100: // imprimir vertices retirada, revisar
            //     // for (int v : g.trilhaEuleriana()) { // se vazia tem q revisar msm
            //     //     cout << v << " ";
            //     // }
            //     // cout << endl;
            //     break;
        }
    }
    return 0;
}
