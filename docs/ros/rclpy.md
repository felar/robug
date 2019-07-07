# Rclpy
`rclpy` ist die Python-Bibliothek, mit der man ROS2 kontrollieren kann. Der Name steht für **R**OS **C**lient **L**ibrary **Py**thon. Hier sind einige der grundlegensten Methoden dieser Bibliothek erklärt. Die detaillierte Dokumentation findet man [hier](http://docs.ros2.org/crystal/api/rclpy/api).

## Allgemein

### `rclpy.init()`
Diese Methode initialisiert `rclpy` und sollte aufgerufen werden, bevor in irgendeiner Weise darauf zugegriffen wird.

**Relevante Argumente:** 

Keine.

### `rclpy.spin()`
Diese Methode führt die Arbeit einer übergebenen Node aus. Die Ausführung des Codes bleibt bei ihr so lange stehen, bis `rclpy` heruntergefahren wird. Diese Methode wird benutzt, um eine erstellte Node "ins Leben zu rufen", und sie so lange aktiv zu halten wie nötig.

**Relevante Argumente:** 

`node=` gibt die Node an, die aktiv werden soll.

### `rclpy.shutdown()`
Diese Methode beendet `rclpy` und alle Nodes, die wir gestartet haben. Sie sollte am Ende des Programms aufgerufen werden.

**Relevante Argumente:** 

Keine.

## Nodes

### Die Klasse `rclpy.node.Node`
Diese Klasse ist der wichtigste Teil an Nodes. Um eine neue Node zu erstellen, leitet man üblicherweise eine eigene Klasse von `rclpy.node.Node` ab.
Wenn man das macht, muss man dem Konstruktor der übergeordneten Klasse den Namen der eigenen Node übergeben, in etwa so:

```python
# Eigene Klasse von rclpy.node.Node ableiten
class MyNode(rclpy.node.Node):

    # Konstruktor
    def __init__(self):

        # Den Konstruktor der Elternklasse aufrufen und den Namen übergeben
        super().__init__(node_name="my_custom_node")
        
        # Im Nachfolgenden werden gewöhnlich Publisher und Subscriber der Node definiert
```

Innerhalb des Codes der eigenen Node kann man alle nachfolgenden Methoden direkt aufrufen (also nicht `Node.bla()` sondern einfach nur `bla()`)

### `Node.create_publisher()`
Diese Methode gibt einem ein Publisher-Objekt zurück, das man von nun an verwenden kann. 

Ein Publisher hat nur eine Methode: `Publisher.publish()`, mit dem Argument `msg=`, mit dem man eine Message publishen kann.

**Relevante Argumente:** 

`msg_type=` gibt den Typ der Messages, die durch diesen Publisher gesendet werden können, an. Dieser Typ muss etwas sein wie `String`. (Siehe Abschnitt über Messages)

`topic=` gibt den Namen des Topics, auf das der Publisher publishen soll an.

### `Node.create_subscription()`
Diese Methode erstellt einen neuen Subscriber. Subscriber haben keine Methoden, die man aufrufen kann.
Stattdessen übergibt man der Methode unter anderem eine Callback-Methode, die aufgerufen wird, wenn eine neue Message auf dem gewünschten Topic gepuplished wird.

Das sieht dann häufig so aus:
```python
def __init__(self):
    super().__init__(node_name='subscriber_node')
    
    # Subscribtion erstellen und Callback-Methode übergeben
    self.subscription = self.create_subscription(msg_type=String, topic='subscribed_topic', callback=self.subscriber_callback)

# Die Callback-Methode wird aufgerufen, wenn eine neue Message gepublished wird
def subscriber_callback(self, msg):
    # Etwas mit der erhaltenen Message anfangen
```

**Relevante Argumente:**

`msg_type=` gibt den Typ der Messages, die durch diesen Subscriber empfangen werden können, an. Dieser Typ muss etwas sein wie `String`. (Siehe Abschnitt über Messages)

`topic=` gibt den Namen des Topics, auf das der Subscriber subscriben soll an.

`callback=` gibt die Methode an, die aufgerufen werden soll, wenn eine neue Message im gegebenen Topic gepublished wird.

## Messages

Messages bzw. Message-Typen müssen eigens importiert werden mittels bspw. `from std_msgs.msg import String`. Alle normalen Message-Typen befinden sich in `std_msgs.msg`. 

Messages haben nur ein Feld: `data`. Über dieses werden die Daten der Message gesetzt und gelesen:
```python
from std_msgs.msg import String

# ...

string_message = String()
string_message.data = "My Message"
```