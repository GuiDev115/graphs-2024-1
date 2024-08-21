#include "./estrutura/Grafos.hpp"
#include <iostream>
#include <sstream>

using namespace std;

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
    Grafo g(numVertices, direcionado_ou_nao_direcionado);

    // Leitura das arestas
    for (int i = 0; i < numAresta; ++i) {
        int arestaID, u, v, w;
        cin >> arestaID >> u >> v >> w;
        g.addAresta(u, v, w);
    }

    int opcao;
    while (iss >> opcao) {
        cout << opcao <<"= Resultado: ";
        switch (opcao) {
            case 1: //conexo
                cout << g.isConexo() << endl;
                break;
            case 2: //bipartido
                cout << g.isBipartido() << endl;
                break;
            case 3: // Euleriano
                cout << g.isEuleriano() << endl;
                break;
            case 4: //Ciclo
                cout << g.possuiCiclo() << endl;
                break;
            case 5: //conexas
                if(g.isDirecionado()){
                    cout << -1 << endl;
                }else{
                    cout << g.componentesConexos() << endl;
                }
                break;
            case 6: // fortemente
                if(!g.isDirecionado()){
                    cout << -1 << endl;
                }else{
                    cout << g.componentesFortementeConexas() << endl;
                }
                break;
            case 7: // imprimir vertices retirada, revisar
                // for (int v : g.trilhaEuleriana()) { // se vazia tem q revisar msm
                //     cout << v << " ";
                // }
                // cout << endl;
                break;
            case 8:
                if (g.isDirecionado()) {
                    cout << -1 << endl;
                } else {
                    for (int v : g.verticesArticulacao()) {
                        cout << v << " ";
                    }
                    cout << endl;
                }
                break;
            case 9: 
                if(g.isDirecionado()){
                    cout << -1 << endl;
                }else {
                    vector<int> pontes = g.arestasPonte();
                    for (int ponte : pontes) {
                        cout << ponte << " ";
                    }
                    cout<<endl;
                }
                break;
            case 10: 
                g.arvoreDFS();
                break;
            case 11: {
                g.arvoreBFS();
                break;
            }
            case 12: {
                g.arvoreGeradoraMinima();
                break;
            }
            case 13: {
                if (g.isDirecionado()) {
                    vector<int> ordem = g.ordemTopologica();
                    std::cout << "Ordem topológica: ";
                    for (int v : ordem) {
                        std::cout << v << " ";
                    }
                    std::cout << std::endl;
                } else {
                    std::cout << "Ordenação topológica não disponível para grafos não direcionados." << std::endl;
                }
                break;
            }
            case 14: {
                int origem = 0, destino = numVertices - 1;
                int caminho = g.caminhoMinimo(origem, destino);
                std::cout << "Caminho mínimo de " << origem << " até " << destino << ": " << caminho << std::endl;
                break;
            }
            case 15: {
                if (g.isDirecionado()) {
                    int origem = 0, destino = numVertices - 1;
                    int fluxo = g.fluxoMaximo(origem, destino);
                    std::cout << "Fluxo máximo de " << origem << " até " << destino << ": " << fluxo << std::endl;
                } else {
                    std::cout << "Fluxo máximo não disponível para grafos não direcionados." << std::endl;
                }
                break;
            }
            case 16: {
                if (g.isDirecionado()) {
                    vector<vector<int>> fecho = g.fechoTransitivo();
                    std::cout << "Fecho transitivo:\n";
                    for (int i = 0; i < fecho.size(); ++i) {
                        for (int j = 0; j < fecho[i].size(); ++j) {
                            std::cout << fecho[i][j] << " ";
                        }
                        std::cout << std::endl;
                    }
                } else {
                    std::cout << "Fecho transitivo não disponível para grafos não direcionados." << std::endl;
                }
                break;
            }
        }
    }
    return 0;
}