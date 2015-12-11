Fyzikální simulace pevného tělesa na GPU
----------------------------------------

Projekt do předmětu GMU - Grafické a multimediální procesory.

Požadavky
=========

Grafická karta s podporou `OpenGL 4.3`.

Kompilace
=========

Projekt využívá knihovny `OpenGL`, `SDL2` a `glew`. Všechny tyto knihovny
jsou multiplatformní, takže by neměl být problém projekt přeložit na Windows.
Nicméně přiložený makefile je pouze pro Linuxové prostředí.

Pro překlad stačí spustit příkaz `make`. Výsledný spustitelný soubor pak je
`bin/rigid-body`.

Spuštení
========

Po překladu spusťte soubor `bin/rigid-body` z adresáře projektu.
Program nepředpokládá žádné vstupní parametry.

<!--
Ovládání
========

Program se ovládá klávesami `W`, `A`, `S`, `D` pro pohyb závislý na směru kamery,
klávesy `SHIFT` a `CTRL` pak slouží pro vertikální posun.

Primárním tlačítkem myši pak lze kamerou rotovat.
-->

Github
======

https://github.com/JOndra91/volumetric-clouds
