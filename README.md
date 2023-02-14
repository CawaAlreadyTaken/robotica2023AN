# robotica2023 

> Progetto di robotica 2023, Daniele Cabassi, Alfredo Ceneri, Stefano Putelli

## Installazione

Gitclonare questa repository nella home del docker fornito a lezione

```sh
$ lab
$ git clone https://github.com/CawaAlreadyTaken/robotica2023.git
```

## Usage

Selezionare il "mondo" decommentando una tra le seguenti righe del file `~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/ur5_generic.py`

```python3
#self.world = None
#self.
#self.
```

Avviare il file con "python3 -i":
```sh
$ python3 -i ~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/ur5_generic.py
```
_(Non nascondiamo l'output e non mettiamo in background perche' siamo interessati ai messaggi di log)_

Da un nuovo terminale, entrare nello stesso docker:
```sh
$ dock-other
```

## Assignments

Gli assignment si trovano sotto i seguenti nomi:

* `ass1`
* `ass2`
* `ass3`
* `ass4`
