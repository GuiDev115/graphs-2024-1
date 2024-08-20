#include "./estrutura/Grafos.hpp"
#include <iostream>

using namespace std;

int main() {
    cout<<"Inicio: "<<endl;
    int numVertices, numAresta;
    string direcionado_ou_nao_direcionado;

    // Leitura do número de vértices e arestas
    cin >> numVertices >> numAresta;

    // Leitura do tipo do grafo (direcionado ou não direcionado)
    cin >> direcionado_ou_nao_direcionado;

    // Inicializa o grafo
    Grafo g(numVertices, direcionado_ou_nao_direcionado);

    // Leitura das arestas
    for (int i = 0; i < numAresta; ++i) {
        int arestaID, u, v, w;
        cin >> arestaID >> u >> v >> w;
        g.addAresta(u, v, w);
    }

    cout << "O grafo e conexo? " << (g.isConexo() ? "Sim" : "Nao") << endl;
    cout << "O grafo e bipartido? " << (g.isBipartido() ? "Sim" : "Nao") << endl;
    cout << "O grafo e Euleriano? " << (g.isEuleriano() ? "Sim" : "Nao") << endl;
    cout << "O grafo possui ciclos? " << (g.possuiCiclo() ? "Sim" : "Nao") << endl;


    return 0;
}