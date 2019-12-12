# Neuronale Netzwerke

## Was sind Neuronale Netzwerke?
Neuronale Netzwerke sind im Groben Einheiten zur Informationsverarbeitung. Sie reagieren jedoch nicht auf jede 
Situtation gleich, sondern ziehen Informationen aus vielen, nicht unbedingt eindeutigen Faktoren und 
verbessern sich selbst indem sie aus Fehlern lernen.
Das Funktionsprinzip von neuronalen Netzwerken wurde von der Natur abgeleitet. Es wurde sich an neuronalen Zellen
orientiert, die, zum Beispiel im Gehirn, angekommene Signale verarbeiten.

## Wie sind künstliche neuronale Netzwerke aufgebaut?
Der Grundbestandteil neuronaler Netzwerke sind die Neuronen. Sie sind in Schichten strukturiert.
Die erste Schicht enthält den Input und die letzte Schicht zeigt die vorhergesagte richtige Antwort des Systems und die 
Wahrscheinlichkeit, dass dies tatsächlich der richtigen Antwort entspricht.
Zwischen dieser Eingabe- und der Ausgabeschicht gibt es sogenannte "Hidden Layers" also verborgene Schichten. Die Anzahl
dieser Schichten und auch die Anzahl der Neuronen pro Schicht sind beliebig, das Bestimmen dieser Anzahlen nennt man Architektur.

### Wie funktionieren Neuronen? 
Jedes Neuron funktioniert durch diese eine mathematische Funktion:

```latex
Y=A(X.W+b)
```

#### Die Eingabe (X)
Die Eingabe wird als Vektor in die Funktion gegeben, auch Bilder werden "flach" in einen Vektor gepackt.
Wenn das neuronale Netzwerk mehrere verborgene Schichten hat, wir die Ausgabe der vorherigen Schicht als Eingabe genutzt.

#### Gewichtungen (W) und Schwellenwerte (b)
Gewichtungen und Schwellenwerte sind die zwei Instanzen, die durch das Lernen des Netzwerkes verändert werden können.
Es gibt so viele Gewichtungen wie Eingaben pro Neuron, sodass die Multiplikation durchgeführt werden kann. 
Es gibt jedoch meist weniger Schwellenwerte, weshalb das + in der Formel zu einem "Broadcast-Plus" wird. 
Es werden also die selben Schwellenwerte mehrfach verwendet.

#### Die Aktivierungsfunktion (A)
Aktivierungsfunktionen sind stark von Neurowissenschaftlern beeinflusst worden. Sie bestimmt das Muster in dem die 
Neuronen aktiviert werden. Diese Funktionen sind immer nonlinear, da ihr Hauptziel ist, die Liniarität des Netzwerkes zu unterbrechen.
Die meistbenutzten Aktivierungsfunktionen sind Sigmoid, relu und softmax.

##### Sigmoid
Die Sigmoid-Funktion wurde früher häufig für die versteckten Schichten verwendet, heute wurde sie aber von effizienteren 
Funktionen abgelöst.

```latex
S(x) = \frac{ 1 }{ 1 + e^{ -x } } = \frac{ e^{ x } }{ e^{ x } + 1}
```

![sigmoid](https://raw.githubusercontent.com/felar/robug/master/pictures_gifs/sigmoid.png)

##### relu
Relu gibt eine 0 aus, wenn der eingegebene Wert kleiner oder gleich 0 ist, gibt aber den exakten eingegeben Wert zurück, wenn das
nicht der Fall ist.
Stark biologisch beeinflusst wurde diese Funktion zum Ablöser der Sigmoid-Funktion, da sie schneller und effizienter funktioniert.

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

### Was genau sagt mir die Ausgabe (Y)?
Die Ausgabe, die die letzte Schicht des Netzwerkes bildet, gibt die Ergebnisse des Netzwerkes aus, also das, was das Netzwerk 
für die richtige Lösung hält.

### Was ist eine Verlustfunktion?
Die Aufgabe der Verlustfunktion ist es die Ergebnisse, die das Netzwerk ausgibt, zu bewerten. 
Sie bildet aus guten und schlechten Ergebnissen einen Wert.

Die richtige Verlustfunktion zu wählen ist sehr schwierig, da sie zu den Anforderungen des Neuronalen Netzwerkes passen muss.
Bei falscher Wahl kann der Prozess des Trainierens länger dauern oder gar nicht erst richtig funktionieren.
 
#### Kreuzentropie
Die Verlustfunktion Kreuzentropie berechnet eine Art Differenz zwischen dem Ergebnis des Netzwerkes und der tatsächlich richtigen Lösung
mit dieser Formel:
 ```latex
-\sum{Y'*log(Y)}
```
Die richtige Lösung ist einem Vektor `Y'` angegeben, bei dem das richtige Ergebnis mit einer 1 und alle falschen Werte mit 
einer 0 markiert sind. Dies nennt man auch One-Hot-Kodierung.

Die Kreuzentropie basiert auf der Idee der Entropie der Informationstheorie, berechnet also die Anzahl der Bits die notwendig sind,
um die Differenz zu übermitteln. Die Funktion gibt eine positive Bitzahl aus.

### Wie funktioniert das Training?
Wenn man mit dem neuronalen Netzwerk anfängt setzt man die Gewichte zufällig. 
Dies führt natürlich am Anfang zu schlechten Ergebnissen. 

Das neuronale Netzwerk minimiert das Ergebnis der Verlustfunktion um eine möglichst 
korrekte Ausgabe auszugeben.
Um diese richtige Ausgabe erreichen zu können gibt es sogenannte Optimierungsalgorithmen, deren Ziel es ist,
den kleinsten Abstand von richtiger Lösung zu der Ausgabe des Netzwerkes zu finden.

Durch das Trainieren des neuronalen Netzwerkes mit Optimierungsalgorithmus wird langsam der gewünschte Tiefpunkt erreicht.

Es werden Werte in das Netzwerk gegeben und das Netzwerk berechnet seine Antwort mit Hilfe der Gewichte und Schwellenwerte 
und der Aktivierungsfunktion.
Diese Antworten werden dann von der Verlustfunktion bewertet und durch das minimieren der Verlustfunktion durch den
Optimierungsalgorithmus, der die Gewichte korrigiert, verbessert.
Das Netzwerk hat dann im nächsten Durchlauf neue Antworten mit den veränderten Gewichten und diese werden dann wiederum von
der Verlustfunktion bewertet, sodass wieder neue Gewichtungen durch die Optimierungsfunktion entstehen.