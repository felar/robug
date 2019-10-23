# Neuronale Netzwerke

## Was sind Neuronale Netzwerke?
Neuronale Netzwerke sind im Groben Einheiten zur Informationsverarbeitung. Sie reagieren jedoch nicht auf jede 
Situtation gleich, sondern ziehen Informationen aus vielen, nicht unbedingt eindeutigen Faktoren und 
verbessern sich selbst indem sie aus Fehlern lernen.
Das Funktionsprinzip von neuronalen Netzwerken wurde von der Natur abgeleitet. Es wurde sich an neuronalen Zellen
orientiert, die, zum Beispiel im Gehirn, angekommene Signale verarbeitet.

## Wie sind künstliche neuronale Netzwerke aufgebaut?
Der Grundbestandteil neuronaler Netzwerke sind die Neuronen. Sie sind in Schichten strukturiert.
Die erste Schicht enthält den Input und die letzte Schicht zeigt die vorhergesagte richtige Antwort des Systems und die 
Wahrscheinlichkeit, dass dies tatächlich der richtigen Antwort entspricht.
Zwischen dieser Input- und der Outputschicht gibt es sogenannte "Hidden Layers" also verborgene Schichten. Die Anzahl
dieser Schichten und auch die Anzahl der Neuronen pro Schicht sind beliebig, das bestimmen dieser Anzahlen nennt man Architektur.

### Wie funktionieren Neuronen? 
Jedes Neuron funktioniert durch diese eine mathematische Funktion:

Y=A(X.W+b)

#### Der Input (X)
Der Input wird als Vektor in die Funktion gegeben, auch Bilder werden "flach" in einen Vektor gepackt.
Wenn das neuronale Netzwerk mehrere verborgene Schichten hat, wir der Output der vorherigen Schicht als Input genutzt.

#### Weights (W) und Biases (b)
Weights, also Gewichte und Biases sind die zwei Instanzen, die durch das Lernen des Netzwerkes verändert werden können.
Es gibt so viele Weights wie Input pro Neuron, sodass die Multiplikation durchgeführt werden kann. 
Es gibt jedoch meist weniger Biases, weshalb das + in der Formel zu einem "Broadcast-Plus" wird. 
Es werden also die selben Biases mehrfach verwendet.

#### Die Aktivierungsfunktion (A)
Aktivierungsfunktionen sind stark von Neurowissenschaftlern beeinflusst worden. Sie bestimmt das Muster in dem die 
Neuronen aktiviert werden. Diese Funktionen sind immer nonlinear, da ihr Haptziel ist, die Liniarität des Netzwerkes zu unterbrechen.
Die meistbenutzten Aktivierungsfunktionen sind Sigmoid, relu und softmax.

##### Sigmoid
Die Sigmoid-Funktion wurde früher häufig für die versteckten Schichten verwendet, heute wurde sie aber von effizienteren 
Funktionen abgelöst.

![Sigmoid_function](https://wikimedia.org/api/rest_v1/media/math/render/svg/9537e778e229470d85a68ee0b099c08298a1a3f6)

##### relu
Stark biologisch beeinflusst wurde diese Funktion zum Ablöser der Sigmoid-Funktion.

![relu_function](https://wikimedia.org/api/rest_v1/media/math/render/svg/5fa5d3598751091eed580bd9dca873f496a2d0ac)

##### softmax
Die Softmax-Funktion wird in der letzten Schicht verwendet. Sie macht die hohen Werte höher und die niedrigen Werte niedriger, sodass
es ein klares Ergebnis geben kann.

![softmax_function](https://wikimedia.org/api/rest_v1/media/math/render/svg/e348290cf48ddbb6e9a6ef4e39363568b67c09d3)

### Was genau sagt mir der Output (Y)?
Der Output, der die letzte Schicht des Netzwerkes bildet
 
![cross_entropy_function](https://www.zahlen-kern.de/editor/equations/ugnm.png)

 ........Hilfe!...... loss function, cross entropy, one hot encoded.......................
 
Das Ziel des neuronalen Netzwerkes ist dann das Ergebnis der Verlustfunktion zu minimieren um einen möglichst 
korrekten Output auszugeben.

Um diesen richtigen Output erreichen zu können gibt es sogenannte Optimierungsalgorithmen, dessen Ziel es ist,
den kleinsten Abstand von richiger Lösung zu dem Output des Netzwerkes zu finden.
Der bekannteste ist das Gradientenverfahren.

##### Gradient Descent, das Gradientenverfahren
Bei diesem Verfahren geht man, von einem Startpunkt anfangend, in Richtung des negativen Gradienten
bis man an einem Punkt angekommen ist, an dem es nicht mehr tiefer geht.
Dazu werden kleine Schritte in die Richtung des Minimums gemacht, die bei uns als Lernrate beizeichnet wird. 
Diese Lernrate darf nicht zu groß gewählt werden, damit man an dem tiefsten Punkt überhaupt ankommen kann.
Bei einer zu großen Lernrate kann man sich das so vorstellen wie bei einem Tal. Man steht auf einem Berg neben
dem Tal, möchte aber in das Tal. Wenn man sich jetzt aber nur in zu großen Schritten bewegen kann, springt man immer 
von einer Seite des Tals auf die andere, erreicht aber nie wirklich sein Ziel.

Durch das Trainieren des neuralen Netzwerkes mit Optimierungsalgorithmus wird langsam der gewünschte Tiefpunkt erreicht.

### Wie funktioniert das Training?
Wenn man mit dem neuronalen Netzwerk anfängt setzt man die Gewichte zufällig. Dies führt natürlich am Anfang zu
schlechten Ergebnissen. Durch die Verlustfunktion und die Optimierungsalgorithmen werden die Gewichte nach 
und nach angepasst.
