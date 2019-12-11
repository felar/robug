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

```latex
Y=A(X.W+b)
```

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

```latex
S(x) = \frac{ 1 }{ 1 + e^{ -x } } = \frac{ e^{ x } }{ e^{ x } + 1}
```

![sigmoid](https://raw.githubusercontent.com/felar/robug/master/pictures_gifs/sigmoid.png)

##### relu
Stark biologisch beeinflusst wurde diese Funktion zum Ablöser der Sigmoid-Funktion.

```latex
f(x) = max(0, x)
```
![relu](https://raw.githubusercontent.com/felar/robug/master/pictures_gifs/relu.png)

##### softmax
Die Softmax-Funktion wird in der letzten Schicht verwendet. Sie macht die hohen Werte höher und die niedrigen Werte niedriger, sodass
es ein klares Ergebnis geben kann.

```latex
softmax(L_{ n }) = \frac{ e^{ L_{ n } } }{ \left\lvert \left\lvert e^{ L } \right\rvert \right\rvert }
```
![softmax](https://raw.githubusercontent.com/felar/robug/master/pictures_gifs/softmax.png)

### Was genau sagt mir der Output (Y)?
Der Output, der die letzte Schicht des Netzwerkes bildet, gibt die Ergebnisse des Netzwerkes aus, also das, was das Netzwerk 
für die richtige Lösung hält.

### Was ist eine Verlustfunktion?
Die Aufgabe der Verlustfunktion ist es die Ergebnisse, die das Netzwerk ausgibt, zu bewerten. 
Sie bildet aus guten und schlechten Ergebnissen einen Wert.

Die richtige Verlustfunktion zu wählen ist sehr schwierig, da sie zu den Anforderungen des Neuronalen Netzwerkes passen muss.
Bei falscher Wahl kann der Prozess des Trainierens länger dauern oder gar nicht erst richtig funktionieren.
 
#### Kreuzentropie
Die Verlustfunktion Kreuzentropie berechnet eine Art Diffenrenz zwischen dem Ergebnis des Netzwerkes und der tatsächlich richtigen Lösung
mit dieser Formel:
 ```latex
-\sum{Y'*log(Y)}
```
Die richtige Lösung ist einem Vektor angegeben, bei dem das richtige Ergebnis mit einer 1 und alle falschen Werte mit 
einer 0 markiert sind. Dies nennt man auch One-Hot-Kodierung.

Die Kreuzentropie basiert auch der Idee der Entropie der Informationstheorie, berechnet also die Anzahl der Bits die notwendig sind,
um die Differenz zu übermitteln. Die Funktion gibt eine positive Bitzahl aus.

___

Das Ziel des neuronalen Netzwerkes ist dann das Ergebnis der Verlustfunktion zu minimieren um einen möglichst 
korrekten Output auszugeben.

Um diesen richtigen Output erreichen zu können gibt es sogenannte Optimierungsalgorithmen, dessen Ziel es ist,
den kleinsten Abstand von richiger Lösung zu dem Output des Netzwerkes zu finden.

Durch das Trainieren des neuralen Netzwerkes mit Optimierungsalgorithmus wird langsam der gewünschte Tiefpunkt erreicht.

### Wie funktioniert das Training?
Wenn man mit dem neuronalen Netzwerk anfängt setzt man die Gewichte zufällig. Dies führt natürlich am Anfang zu
schlechten Ergebnissen. Durch die Verlustfunktion und die Optimierungsalgorithmen werden die Gewichte nach 
und nach angepasst.