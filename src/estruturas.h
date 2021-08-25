#ifndef POSICOES_H
#define POSICOES_H

enum Direcao
{
  NORTE,
  LESTE,
  SUL,
  OESTE
};

enum Comando
{
  CONTINUAR,
  FRENTE,
  ESQUERDA,
  DIREITA
};

struct Posicao
{
  int x;
  int y;
};

#endif