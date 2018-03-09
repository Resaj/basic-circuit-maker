# Generador de circuitos
Generador de circuitos para las pruebas de siguelíneas en la modalidad de velocistas.

Este programa ha sido ideado para facilitar la creación de este tipo de circuitos en mapa de bits y evitar invertir horas con programas de diseño gráfico para cada circuito.

Para hacer uso del programa basta con tener instalado Octave o Matlab.

Si se abre la carpeta "software", podrán identificarse varios archivos con extensión ".m". El programa principal se encuentra en "basic_circuit_maker.m", mientras que el resto de archivos son ejemplos de circuitos que he ido probando para testear el programa.

El primer paso es identificar las dimensiones de la lona y los parámetros del trazado (coordenadas y dirección del origen, y coordenadas absolutas de los tramos). Estos datos han de introducirse en un archivo como los de los ejemplos de los circuitos, que luego será llamado como una función desde "basic_circuit_maker.m".

Hay que tener en cuenta que para llamar a una función desde "circuit_maker.m", el nombre de la función tiene que ser el nombre del archivo y aparecer en la cabecera del archivo. Por ejemplo, el archivo "coord_mgw2015.m" tiene en su cabecera "function [dim_cto origen_cto tramos_cto] = coord_mgw2015()" y se llama desde "basic_circuit_maker.m" con la línea "[dim origen tramos] = coord_mgw2015();".

Una vez establecidos los parámetros del circuito y modificada la llamada de la función en "basic_circuit_maker.m", sólo queda comprobar que se ejecuta correctamente. Para no perder tiempo de ejecución si saliese mal el circuito, se comprueba antes que el trazado central ha sido especificado correctamente y que el trazado límite no se sale de la lona (las esquinas de la lona quedan marcadas con puntos en las gráficas). Esto se realiza dando el valor "1" a las variables "representar_trazado_central" y "representar_trazado_limite" al principio de "basic_circuit_maker.m".

<p align="center">
<img src="images/Trazado central.png" width="600" align = "center">
</p>

<p align="center">
<img src="images/Trazado limite.png" width="600" align = "center">
</p>

Cuando se haya comprobado la simulación gráfica del circuito, puede volver a darse el valor "0" a las variables mencionadas anteriormente para no ejecutar más las gráficas.

Ahora queda generar el circuito y mostrarlo, lo cual se hace habilitando a "1" las variables "generar_circuito" y "mostrar_circuito".

<p align="center">
<img src="images/circuito.bmp" width="600" align = "center">
</p>

La resolución del circuito está por defecto en 0.5 píxeles/mm para no demorar demasiado su generación, pero se ha llegado a probar con 4 píxeles/mm en un circuito de 5x2m, obteniendo un bmp de 457MB en aproximadamente 1 minuto de ejecución.

Si se ejecuta el programa en Octave desde un terminal de Ubuntu en vez de ejecutarlo desde la interfaz gráfica, es aconsejable descomentar las dos últimas lineas de "basic_circuit_maker.m", las cuales obligan a pulsar enter para finalizar el programa. De lo contrario es posible que se cierren todas las gráficas generadas en cuanto termine la ejecución y no puedan visualizarse.

## Autor
[Rubén Espino San José](https://github.com/Resaj)

## Licencia
<p align="center">
<img src="license/by-sa.png" align = "center">
</p>

Todos estos productos están liberados mediante [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/).  
_All these products are released under [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/)._
