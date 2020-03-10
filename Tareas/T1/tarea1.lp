% programa destinado a resolver el minijuego connect del juego Pou


% un ejemplo de instancia (no debe compilarse en el programa final)

%fila(1..5).  % Ymax
%columna(1..5).  % Xmax

%color(c1, yorigen1, xorigen1)
%color(verde, 3, 1).  
%color(verde, 4, 1).
%color(azul, 5, 1).
%color(azul, 5, 3).
%color(blanco, 5, 4).
%color(blanco, 4, 5).
%color(amarillo, 3, 3).
%color(amarillo, 3, 4).
%color(morado, 3, 5).
%color(morado, 2, 4).
%----
%fila(1..15). columna(1..15).

%color(blanco, 1, 4).
%color(blanco, 15, 10).
%color(verde_oscuro, 2, 8).
%color(verde_oscuro, 5, 2).
%color(cafe_claro, 1, 9).
%color(cafe_claro, 2, 10).
%color(rosado_claro, 1, 14).
%color(rosado_claro, 2, 9).
%color(morado, 1, 15).
%color(morado, 8, 9).
%color(azul_claro, 2, 7).
%color(azul_claro, 4, 3).
%color(naranjo, 3, 8).
%color(naranjo, 10, 13).
%color(celeste, 3, 14).
%color(celeste, 10, 5).
%color(rojo, 3, 15).
%color(rojo, 12, 15).
%color(cafe_oscuro, 5, 7).
%color(cafe_oscuro, 6, 5).
%color(rosado_oscuro, 5, 15).
%color(rosado_oscuro, 7, 15).
%color(turquesa, 6, 4).
%color(turquesa, 14, 14).
%color(turquesa, 6, 4).
%color(turquesa, 14, 14).
%color(azul, 7, 12).
%color(azul, 11, 13).
%color(amarillo, 8, 8).
%color(amarillo, 9, 7).
%color(verde, 9, 13).
%color(verde, 12, 14).
%color(gris, 12, 8).
%color(gris, 13, 3).

% fin ejemplo instancia


% Reglas de integridad:

% las casillas que son parte de un camino están coloreadas por ese camino
coloreada(C, Y, X) :- camino(C, Y, X, Y + 1, X).
coloreada(C, Y, X) :- camino(C, Y, X, Y - 1, X).

coloreada(C, Y, X) :- camino(C, Y, X, Y, X + 1).
coloreada(C, Y, X) :- camino(C, Y, X, Y, X - 1).


% definir set colores
set_colores(C) :- color(C, Y, X), fila(Y), columna(X).

% no puede ocurrir que exista una casilla no coloreada o colorear algo que no es una casilla
:- not coloreada2(Y, X), fila(Y), columna(X).
:- coloreada2(Y, X), not fila(Y), columna(X).
:- coloreada2(Y, X), fila(Y), not columna(X).
:- coloreada2(Y, X), not fila(Y), not columna(X).
coloreada2(Y, X) :- coloreada(C, Y, X).
% cada casilla está coloreada de un solo color
%1{coloreada(C, Y, X): set_colores(C)}1 :- fila(Y), columna(X).

% no pueden haber una casilla de dos colores.
:- coloreada(C1, Y, X), coloreada(C2, Y, X), fila(Y), columna(X), C1 != C2.

% definir casillas no iniciales
inicial(Y, X) :- color(C, Y, X). %, set_colores(C), fila(Y), columna(X)
1{no_inicial(C, Y, X) : set_colores(C)}1 :- not inicial(Y, X), fila(Y), columna(X).


% Predicado debe ser de la forma camino(Color, Yorigen, Xorigen, Ydestino, Xdestino)

% hay un camino si las casillas de al lado estan coloreadas del mismo color
% ESTO NO SIEMPRE SE CUMPLE
% camino(C, Y, X, Y+1, X) :- coloreada(C, Y, X), coloreada(C, Y+1, X).
% camino(C, Y, X, Y, X+1) :- coloreada(C, Y, X), coloreada(C, Y, X+1).
% camino(C, Y, X, Y-1, X) :- coloreada(C, Y, X), coloreada(C, Y-1, X).
% camino(C, Y, X, Y, X-1) :- coloreada(C, Y, X), coloreada(C, Y, X-1).

% cada casilla inicial tiene un camino y cada casilla no inicial tiene dos caminos.
1{camino(C, Y, X, Y+1, X);camino(C, Y, X, Y-1, X);camino(C, Y, X, Y, X+1);camino(C, Y, X, Y, X-1)}1 :- color(C, Y, X).
2{camino(C, Y, X, Y+1, X);camino(C, Y, X, Y-1, X);camino(C, Y, X, Y, X+1);camino(C, Y, X, Y, X-1)}2 :- no_inicial(C, Y, X).

% caminos bidireccionales
:- not camino(C, Y+1, X, Y, X), camino(C, Y, X, Y+1, X).
:- not camino(C, Y, X+1, Y, X), camino(C, Y, X, Y, X+1). 
:- not camino(C, Y-1, X, Y, X), camino(C, Y, X, Y-1, X).
:- not camino(C, Y, X-1, Y, X), camino(C, Y, X, Y, X-1). 

% una casilla alcanza_inicio si existe un camino o si ambas casillas pueden alcanzar una tercera
% OPTIMIZABLE
alcanza_inicio(C, Y, X) :- camino(C, Y, X, Y + 1, X), color(C, Y + 1, X).
alcanza_inicio(C, Y, X) :- camino(C, Y, X, Y - 1, X), color(C, Y - 1, X).
alcanza_inicio(C, Y, X) :- camino(C, Y, X, Y, X + 1), color(C, Y, X + 1).
alcanza_inicio(C, Y, X) :- camino(C, Y, X, Y, X - 1), color(C, Y, X - 1).

alcanza_inicio(C, Y, X) :- camino(C, Y, X, Y + 1, X), alcanza_inicio(C, Y + 1, X).
alcanza_inicio(C, Y, X) :- camino(C, Y, X, Y - 1, X), alcanza_inicio(C, Y - 1, X).
alcanza_inicio(C, Y, X) :- camino(C, Y, X, Y, X + 1), alcanza_inicio(C, Y, X + 1).
alcanza_inicio(C, Y, X) :- camino(C, Y, X, Y, X - 1), alcanza_inicio(C, Y, X - 1).

% casillas iniciales por si mismas (para evitar condición casillas distintas)
% alcanzable(C, Y, X, Y, X) :- color(C, Y, X).

% casilla coloreada de un color debe poder alcanzar el inicio y el final.
:- not alcanza_inicio(C, Y, X), coloreada(C, Y, X).

#show camino/5.
