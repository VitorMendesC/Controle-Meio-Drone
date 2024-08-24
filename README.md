# Controlador de meio drone
Controlador PID discreto implementado em STM32 NUCLEO-F401RE com planejador de trajetória.
A planta é um meio drone exibido na Figura 1.

<p align="center">
Figura 1 - Meio drone.
</p>

![](img/plant.jpg)

A planta em malha fechada pode ser vista na Figura 2.

<p align="center">
Figura 2 - Planta em malha fechada.
</p>

![](img/control.drawio.png)


Onde F1 é a função que converte a saída do controlador dada em Newtons para o ciclo de trabalho correspondente que deve ser fornecido ao driver. De forma análoga F2 é a função que converte a leitura do potênciomentro de posição de tensão para radianos.

A planta foi modelada através da resposta a entrada degrau, através da qual sabe-se que possui a forma
    $H(s) = \frac{K}{Js^2+Bs} * \frac{A}{s}$


## Algoritimo de planejamento de rota


## Apresentação final
A apresentação contendo os desenvolvimentos e resultados pode ser vista <a href="Apresentação_Meio_Drone_final.pdf" class="image fit">aqui</a>.

## Video demo
Um video que demonstra a resposta ao acompanhamento de trajetória na presença de distúrbio pode ser visto <a href="https://youtube.com/shorts/MJpE3WVZWKM" class="image fit">aqui</a>.

