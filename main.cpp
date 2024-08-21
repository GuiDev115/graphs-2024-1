#include "./estrutura/Grafos.hpp"
#include <iostream>
#include <sstream>

using namespace std;

int main() {
    cin.ignore();
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
    Grafo g(numVertices, direcionado_ou_nao_direcionado);

    // Leitura das arestas
    for (int i = 0; i < numAresta; ++i) {
        int arestaID, u, v, w;
        cin >> arestaID >> u >> v >> w;
        g.addAresta(u, v, w);
    }

    int opcao;
    while (iss >> opcao) {
        switch (opcao) {
            case 1:
                cout << g.isConexo() << endl;
                break;
            case 2:
                cout << g.isBipartido() << endl;
                break;
            case 3:
                cout << g.isEuleriano() << endl;
                break;
            case 4:
                cout << g.possuiCiclo() << endl;
                break;
            case 5:
                if(g.isDirecionado()){
                    cout << -1 << endl;
                }else{
                    cout << g.componentesConexos() << endl;
                }
                break;
            case 6:
                if(!g.isDirecionado()){
                    cout << -1 << endl;
                }else{
                    auto sccs = g.componentesFortementeConexas();
                    cout << "Componentes Fortemente Conexas:" << endl;
                    for (const auto& scc : sccs) {
                        for (int v : scc) {
                            cout << v << " ";
                        }
                        cout << endl;
                    }
                }
                break;
            }
    }
    return 0;
}