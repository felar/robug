# Optimierungsalgorithmen
Optimierungsalgorithmen versuchen die Verlustfunktion möglichst klein zu bekommen. Das tun sie, indem sie 
die Gewichte verändern, sodass die Vorhersagen des Netzwerkes möglichst richtig werden.

Ein gutes Beispiel dafür ist ein Wanderer, der, ohne etwas zu sehen, einen Berg hinunter kommen möchte.
Es ist unmöglich für ihn zu wissen in welche Richtung er gehen sollte, aber er spürt, ob er bergab (Verbesserung) 
oder bergauf (Verschlechterung) geht. Wenn er immer weiter bergab geht, kommt er irgendwann im Tal an.

## Gradient Descent, das Gradientenverfahren
Dieses Verfahren ist der populärste Algorithmus für Maschine Learning. 
Bei diesem Verfahren geht man, von einem Startpunkt anfangend, in Richtung des negativen Gradienten
bis man an einem Punkt angekommen ist, an dem es nicht mehr tiefer geht.
Die Gradienten repräsentieren dabei, wie sich die Verlustfunktion verändert, wenn die Gewichte minimal verändert
würden.
Es werden kleine Schritte in die Richtung des Minimums gemacht, die bei uns als Lernrate beizeichnet wird. 
Diese Lernrate darf nicht zu groß gewählt werden, damit man an dem tiefsten Punkt überhaupt ankommen kann.

Bei einer zu großen Lernrate kann man sich das so vorstellen wie bei dem Wanderer und dem Tal. 
Er steht auf dem Berg neben dem Tal und möchte weiterhin in das Tal. 
Wenn er sich jetzt aber nur in zu großen Schritten bewegen kann, springt er immer 
von einer Seite des Tals auf die andere, erreicht aber nie wirklich sein Ziel.

![gradient_descent](https://raw.githubusercontent.com/felar/robug/master/pictures_gifs/gradientdescent.gif)
#
Im Vergleich zu anderen Algorithmen ist das Gradientenverfahren jedoch sehr langsam.
Deshalb wurden zunächst einige kleine Abänderungen des Verfahrens genutzt und später dann auch ganz andere Algorithmen.

![comparison](https://raw.githubusercontent.com/felar/robug/master/pictures_gifs/optimizer_comparison.gif)

## Stochastisches Gradientenverfahren (SGD)
Das Standart Gradientenverfahren braucht sehr viele Berechnungen bevor etwas passiert. Das stochastische Gradientenverfahren
kürzt diesen Prozess drastisch. "Stochastisch" steht in diesem Fall einfach für "zufällig". 
Das Verfahren wählt zufällig genau einen Wert des Outputs pro Durchgang und berechnet nur für diesen den Gradienten, anstatt
für jeden Wert.

## Gradientenverfahren mit Momentum
Das Gradientenverfahren mit Momentum ist erstmal so wie das SGD aufgebaut. Es gibt jedoch eine "durchschnittliche Bewegung", die 
Momentum genannt wird und für die Berechnung des Gradienten genutzt wird.

## Nesterovs beschleunigtes Gradientenverfahren (NAG)


## Adagrad

## Adadelta

## RMSprop
Der RMSprop Algorithmus, den wir in unserem Projekt benutzen, funktioniert 



