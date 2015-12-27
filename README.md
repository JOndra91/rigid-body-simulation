Fyzikální simulace pevného tělesa na GPU
----------------------------------------

Projekt do předmětu GMU - Grafické a multimediální procesory.


Požadavky
=========

Grafická karta s podporou `OpenGL 4.3`.


Kompilace
=========

Projekt využívá externě knihovny `OpenGL`, `SDL2` a `glew`. Všechny tyto knihovny
jsou multiplatformní, takže by neměl být problém projekt přeložit na Windows.
Nicméně přiložený makefile je pouze pro Linuxové prostředí.

Pro překlad stačí spustit příkaz `make`. Výsledný spustitelný soubor pak je
`bin/gmu`.

Přibalený makefile využívá program `cmake`, který je potřeba pro sestavení programu.
V případě problémů zkuste použít minimálně verzi CMake `3.0.2`.

Dále je potřeba mít nainstalovaný `python3`, který se používá pro převod
zdrojového kódu OpenCL kernelu do C stringu, aby bylo možné kernel přikompilovat
přímo do knihovny.


Spuštení
========

Po překladu spusťte soubor `bin/gmu` z adresáře projektu.

Následujícími parametry lze nastavit OpenCL akceleraci:

* --none: Program nepoužije OpenCL akceleraci
* --cpu: Program použije OpenCL akceleraci na CPU
* --gpu: Program použije OpenCL akceleraci na GPU


Ovládání
========

Program se ovládá klávesami `W`, `A`, `S`, `D` pro pohyb závislý na směru kamery,
klávesy `SHIFT` a `CTRL` pak slouží pro vertikální posun.

Primárním tlačítkem myši pak lze kamerou rotovat.


Github
======

https://github.com/JOndra91/rigid-body-simulation
