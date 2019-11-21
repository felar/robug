# Reinforcement Learning

## Wofür Reinforcement Learning?
Im bisherigen Beispiel zu neuronalen Netzwerken sind wir immer davon
ausgegangen, dass wir einen Datensatz haben, in dem wir sowohl einen Input
als auch eine zugeordnete Antwort haben, von der wir wissen, dass sie richtig
ist. Davon hängt insbesondere unsere Loss Function ab.

Was aber, wenn wir diese Informationen gar nicht haben? Man kann nicht für jedes
Problem bei einem gegebenen Input genau sagen, was in dieser Situation das "perfekte"
Verhalten ist. Ein grundlegendes neuronales Netz eignet sich gut für Klassifizierungsprobleme,
aber in der Zielsetzung dieses Projekts ist diese Architektur nicht sonderlich geeignet.
Denn wir haben keinen Datensatz mit bekannten Labels, und das neuronale Netzwerk braucht
außerdem ein gewisses "Zeitgefühl", muss also auch einen verzögerten Gewinn erkennen und
anstreben können. Außerdem gilt ein Erfolg (oder Fehlverhalten) des Netzwerks ja nicht nur für den Schritt,
der unmittelbar zum Ergebnis geführt hat, sondern die Schritte davor waren ebenfalls
relevant, wenn nicht sogar viel wichtiger als der letzte Schritt.

All das sind Limitationen von grundlegenden neuronalen Netzwerken, die sie für ein Problem
wie "Nicht gegen Wände fahren" ungeeignet macht. Es braucht eine Abwandlung der klassischen
Architektur: Das sogenannte _Reinforcement Learning_.

## Besonderheiten von Reinforcement Learning
Das Herzstück von maschinellem Lernen bleibt auch beim Reinforcement Learning gleich:
Wir haben noch immer ein Netzwerk bestehend aus Neuronen, mit einer oder mehreren Schichten.
Auch sonst bleibt viel gleich: Wir benutzen weiterhin Activation Functions (_Relu_ alle
Schichten bis auf die letzte, in der _Softmax_ verwendet wird). Der entscheidende Unterschied
liegt in der Loss Function, und der Art, wie das Netzwerk trainiert wird.

### Wie beim Reinforcement Learning trainiert wird
Bei einem klassischen neuronalen Netzwerk kann man praktisch nach jedem Mal, dass das
Netzwerk ein Ergebnis produziert hat, die Optimierungsalgorithmus das Netzwerk durchlaufen
lassen. Das ist beim Reinforcement Learning nicht möglich, da wir oft das Ergebnis einer
Aktion erst im Nachhinein erfahren. Also wird anders trainiert: Wenn das neuronale Netzwerk
beispielsweise ein Spiel spielen soll, dann lässt man es (ohne es während des Spiels zu ändern)
eine komplette Runde spielen, und speichert für jeden "Schritt" innerhalb des Spiels die
Anzahl der gesammelten Punkte. Die Punkte kommen aus einer Bewertung, wie sich das Netzwerk
verhalten hat, und müssen vorher festgesetzt werden. Für ein Spiel könnte so eine Bewertung
beispielsweise so aussehen:

| Aktion         | Punkte    |
| -------------- | --------: |
| Tor schiessen  | + 20      |
| Tor kassieren  | - 20      |
| Ball bekommen  | + 5       |
| Ball verlieren | - 5       |
| Spiel gewinnen | + 100     |
| Spiel verlieren| - 100     |

Von der Wahl so einer Tabelle hängt dann auch das Verhalten des Netzwerks ab. Die 
Belohnung (Reward [R]) eines Schritts fliesst dann als Faktor in die Loss Function (basierend
auf Cross Entropy) ein:

```latex
loss = -R_{ i } * \sum{ Y'_{ i } * log(Y_{ i }) }
```

Erst nachdem wir in mindestens einer Runde die Belohnungen angesammelt haben, versuchen
wir, das Netzwerk zu optimieren. Wir berechnen das Ergebnis der Loss Function aber trotzdem
für jeden Schritt im Spiel einzeln.

Das größte Problem in der Loss Function haben wir aber noch nicht gelöst: Wir haben keine
Angaben, die wir für die Labels einsetzen können! Wir haben durch das Einführen der Belohnungstabelle
jetzt aber einen Vorteil: Wir müssen nicht die Differenz zwischen dem vorgegebenen Wert und
dem tatsächlichen Wert berechnen, um die Qualität des Ergebnisses des Netzwerks zu berechnen.
Denn wir haben die Wertung ja bereits dadurch, dass die Belohnungstabelle positive und negative
Belohnungen verteilt! Wir bewerten also nur die Qualität der Aktion, die gespielt wurde,
ohne eine bessere anzugeben. Und das machen wir ganz einfach, indem wir für [P'] die Aktion
(wieder "one-hot-encoded") angeben, die am Ende tatsächlich gespielt wurde. Da unser
neuronales Netzwerk ja Wahrscheinlichkeiten ausspuckt, finden wir diesen, indem wir
"per Zufall" (natürlich gewichtet durch das Ergebnis des Netzwerks) eine Aktion auswählen.
Diese gewählte Aktion wird dann aus Sicht des Optimierungsalgorithmus wie eine Konstante
behandelt.

### Belohnungen verlieren ihren Wert
Es gibt noch weitere Modifikationen an der Loss Function, die nötig sind, damit das
Trainieren des Netzwerks in einem vernünftigen zeitlichen Rahmen möglich wird. Die erste
ist dabei, dass Belohnungen über längere Zeit ihren Wert verlieren. Wenn unser Netzwerk
beispielsweise gerade einen Punkt verloren hat, sind die Aktionen, die es direkt vorher
gewählt hat, wahrscheinlich entscheidender für diese Belohnung, als Aktionen, die schon
eine Weile her sind. Daher verringert man die Menge der Belohnung exponentiell. Man
wählt einen Hyperparameter, wie stark die Belohnungen über mehrere Schritte abnehmen sollen
(beispielsweise 0.5). Dann berechnet man die eigentliche Belohnung für einen Schritt
beim Index [i] und Abnahmefaktor [d] wie folgt:

[R_i = R*d^i]


// TODO: Normalization mod