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
Zwischen dieser Input- und der Outputschicht gibt es sogenannte "Hidden Layers" also versteckte Schichten. Die Anzahl
dieser Schichten und auch die Anzahl der Neuronen pro Schicht sind beliebig, das bestimmen dieser Anzahlen nennt man Architektur.

### Wie funktionieren Neuronen? 
Jedes Neuron funktioniert durch diese eine mathematische Funktion:

Y=A(X.W+b)

#### Der Input (X)
Der Input wird als Vektor in die Funktion gegeben, auch Bilder werden "flach" in einen Vektor gepackt.
Wenn das neuronale Netzwerk mehrere versteckte Schichten hat, wir der Output der vorherigen Schicht als Input genutzt.

#### Weights (W) und Biases (b)
Weights und biases sind die zwei Instanzen, die durch das Lernen des Netzwerkes verändert werden können.
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
![Sigmoid function_graph](https://upload.wikimedia.org/wikipedia/commons/thumb/8/88/Logistic-curve.svg/640px-Logistic-curve.svg.png)

##### relu
Stark biologisch beeinflusst wurde diese Funktion zum Ablöser der Sigmoid-Funktion.

![relu_function](https://wikimedia.org/api/rest_v1/media/math/render/svg/5fa5d3598751091eed580bd9dca873f496a2d0ac)
![relu_function_graph](https://upload.wikimedia.org/wikipedia/commons/thumb/f/fe/Activation_rectified_linear.svg/440px-Activation_rectified_linear.svg.png)

##### softmax
Die Softmax-Funktion wird in der letzten Schicht verwendet. Sie macht die hohen Werte höher und die niedrigen Werte niedriger, sodass
es ein klares Ergebnis geben kann.

![softmax_function](https://wikimedia.org/api/rest_v1/media/math/render/svg/e348290cf48ddbb6e9a6ef4e39363568b67c09d3)
![softmax_function_graph]()

