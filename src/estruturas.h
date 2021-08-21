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
  ESQUERDA,
  FRENTE,
  DIREITA,
  CONTINUAR
};

struct Posicao
{
  int x;
  int y;
};

#endif