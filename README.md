# robotica2023AN

> Progetto di robotica 2023, Daniele Cabassi, Alfredo Ceneri, Stefano Putelli

## Installazione

Gitclonare questa repository nella home del docker fornito a lezione

```sh
$ lab
$ git clone https://github.com/CawaAlreadyTaken/robotica2023AN.git
```

## Usage

Copiare la cartella "worlds" di questa repo nella `ros_ws`:

```sh
$ cp -r ~/robotica2023AN/worlds ~/ros_ws/src/locosim/ros_impedance_controller/
```

Selezionare il "mondo" inserendo una tra le seguenti righe in fondo al costruttore della classe "Ur5Generic" nel file `~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/ur5_generic.py`

```python3
self.world = 'assignment1.world'
self.world = 'assignment2.world'
self.world = 'assignment3.world'
self.world = 'assignment4.world'
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

Dare il catkin_make per compilare e linkare i file necessari:
```sh
$ cd ~/robotica2023AN
$ catkin_make
```

## Assignments

Il controllo del robot per lo svolgimento degli assignment e' scritto in cpp. I file si trovano in `~/robotica2023AN/src/control/src/`.
Una volta impostato il nome del ".world" corretto ed avviato il file python `ur5_generic.py` (vedi sopra) e' necessario avviare il nodo per la vision:
```sh
$ source ~/robotica2023AN/devel/setup.bash
$ python3 ~/robotica2023AN/src/vision/src/visionNode.py >/dev/null &
```

A questo punto e' sufficiente avviare l'eseguibile compilato dal cpp corretto. Scegliere tra:

* `rosrun control assignment1`
* `rosrun control assignment2`
* `rosrun control assignment3`
* `rosrun control assignment4`

In Gazebo, il braccio dovrebbe ora muoversi rispettando le consegne dei relativi assignment.
