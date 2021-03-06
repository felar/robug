# Bestärkendes Lernen

## Wofür Bestärkendes Lernen?
Im bisherigen Beispiel zu neuronalen Netzwerken sind wir immer davon
ausgegangen, dass wir einen Datensatz haben, in dem wir sowohl eine Eingabe
als auch eine zugeordnete Antwort haben, von der wir wissen, dass sie richtig
ist. Davon hängt insbesondere unsere Verlustfunktion ab.

Was aber, wenn wir diese Informationen gar nicht haben? Man kann nicht für jedes
Problem bei einer gegebenen Eingabe genau sagen, was in dieser Situation das "perfekte"
Verhalten ist. Ein grundlegendes neuronales Netz eignet sich gut für Klassifizierungsprobleme,
aber in der Zielsetzung dieses Projekts ist diese Architektur nicht sonderlich geeignet.
Denn wir haben keinen Datensatz mit bekannten Antworten, und das neuronale Netzwerk braucht
außerdem ein gewisses "Zeitgefühl", muss also auch einen verzögerten Gewinn erkennen und
anstreben können. Außerdem gilt ein Erfolg (oder Fehlverhalten) des Netzwerks ja nicht nur für den Schritt,
der unmittelbar zum Ergebnis geführt hat, sondern die Schritte davor waren ebenfalls
relevant, wenn nicht sogar viel wichtiger als der letzte Schritt.

All das sind Limitationen von grundlegenden neuronalen Netzwerken, die sie für ein Problem
wie "Nicht gegen Wände fahren" ungeeignet macht. Es braucht eine Abwandlung der klassischen
Architektur: Das sogenannte _Bestärkende Lernen_.

## Besonderheiten von Bestärkendem Lernen
Das Herzstück von maschinellem Lernen bleibt auch beim Bestärkenden Lernen gleich:
Wir haben noch immer ein Netzwerk bestehend aus Neuronen, mit einer oder mehreren Schichten.
Auch sonst bleibt viel gleich: Wir benutzen weiterhin Aktivierungsfunktionen (_Relu_ alle
Schichten bis auf die letzte, in der _Softmax_ verwendet wird). Der entscheidende Unterschied
liegt in der Verlustfunktion, und der Art, wie das Netzwerk trainiert wird.

### Wie beim Bestärkenden Lernen trainiert wird
Bei einem klassischen neuronalen Netzwerk kann man praktisch nach jedem Mal, dass das
Netzwerk ein Ergebnis produziert hat, die Optimierungsalgorithmus das Netzwerk durchlaufen
lassen. Das ist beim Bestärkenden Lernen nicht möglich, da wir oft das Ergebnis einer
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
Belohnung (Reward `R`) eines Schritts fliesst dann als Faktor in die Verlustfunktion (basierend
auf der Kreuzentropie) ein:

```latex
loss = -R_{ i } * \sum{ Y'_{ i } * log(Y_{ i }) }
```

Erst nachdem wir in mindestens einer Runde die Belohnungen angesammelt haben, versuchen
wir, das Netzwerk zu optimieren. Wir berechnen das Ergebnis der Verlustfunktion aber trotzdem
für jeden Schritt im Spiel einzeln.

Das größte Problem in der Verlustfunktion haben wir aber noch nicht gelöst: Wir haben keine
Angaben, die wir für die korrekten Antworten einsetzen können! Wir haben durch das Einführen der Belohnungstabelle
jetzt aber einen Vorteil: Wir müssen nicht die Differenz zwischen dem vorgegebenen Wert und
dem tatsächlichen Wert berechnen, um die Qualität des Ergebnisses des Netzwerks zu berechnen.
Denn wir haben die Wertung ja bereits dadurch, dass die Belohnungstabelle positive und negative
Belohnungen verteilt! Wir bewerten also nur die Qualität der Aktion, die gespielt wurde,
ohne eine bessere anzugeben. Und das machen wir ganz einfach, indem wir für `P'` die Aktion
(wieder "One-Hot-Kodiert") angeben, die am Ende tatsächlich gespielt wurde. Da unser
neuronales Netzwerk ja Wahrscheinlichkeiten ausspuckt, finden wir diesen, indem wir
"per Zufall" (natürlich gewichtet durch das Ergebnis des Netzwerks) eine Aktion auswählen.
Diese gewählte Aktion wird dann aus Sicht des Optimierungsalgorithmus wie eine Konstante
behandelt.

### Belohnungen verlieren ihren Wert
Es gibt noch weitere Modifikationen an der Verlustfunktion, die nötig sind, damit das
Trainieren des Netzwerks in einem vernünftigen zeitlichen Rahmen möglich wird. Die erste
ist dabei, dass Belohnungen über längere Zeit ihren Wert verlieren. Wenn unser Netzwerk
beispielsweise gerade einen Punkt verloren hat, sind die Aktionen, die es direkt vorher
gewählt hat, wahrscheinlich entscheidender für diese Belohnung, als Aktionen, die schon
eine Weile her sind. Daher verringert man die Menge der Belohnung exponentiell. Man
wählt einen Hyperparameter, wie stark die Belohnungen über mehrere Schritte abnehmen sollen
(beispielsweise 0.5). Dann berechnet man die eigentliche Belohnung für einen Schritt
beim Index `i` und Abnahmefaktor `d` wie folgt:

```latex
R_{i} = R * d^{i}
```


### Belohnungen werden normalisiert
Besonders am Anfang des Trainings trifft ein Netzwerk noch essentiell zufällige Entscheidungen.
Das heißt, es gibt viele Entscheidungen, die negativ bewertet werden, dagegen aber sehr wenige,
die positiv bewertet werden. Es hat sich in Experimenten gezeigt, dass Netzwerke sich schneller
trainieren lassen, wenn man bei diesen seltenen gut bewerteten Entscheidungen eine (verhältnismäßig)
erhöhte Belohnung gibt, indem man die Belohnungen innerhalb eines Trainingsschritts (manchmal auch
kleinere Einheiten, genannt "Batches") normalisiert.

```latex
R_{i} = \frac{R_{i} - \overline{R}}{stdev(R)}
```

## Ausblick
Natürlich sind die Möglichkeiten des bestärkenden Lernens mit den hier gezeigten Methoden noch
lange nicht ausgeschöpft - das Feld der Architekturen für bestärkendes Lernen ist sehr aktuell 
und aktiv, und jedes Jahr werden neue Architekturen mit neuen Ideen veröffentlicht. Zwei dieser
Architekturen, die momentan sehr populär sind, ist das Deep Q Learning und das Modell (Soft) Actor
Critic.

_Notiz: Gerade diese beiden Methoden sind natürlich auch mathematisch weitaus komplexer als hier
dargestellt, jedoch soll dieser Ausblick lediglich eine Perspektive geben, in welche Richtung sich
das Feld aktuell entwickelt._

### Deep Q Learning
Beim Deep Q Learning (auch **DQN**) wird das neuronale Netzwerk nicht darauf trainiert, sich für
eine Aktion zu entscheiden. Stattdessen soll es den Platz einer Wertefunktion (Q-Funktion) annehmen - das heißt,
eine Funktion, die für eine gegebene Beobachtung und eine Aktion die für diese Aktion zu erwartende
Belohnung berechnet. Die Aktion, die dann gewählt wird, hängt von davon ab, von welcher Aktion das Netzwerk
sich die höchste Belohnung verspricht. Das Netzwerk wird dann darauf trainiert, die Belohnungen möglichst
akkurat voraus zu sagen.

Bei dieser Art von Lernen gibt es noch einen weiteren Vorteil: Man kann vorher gesammelte Daten
beim Training wiederverwenden, und muss sie nicht nach jedem Trainingsvorgang komplett neu sammeln. Das nennt
man auch "off-policy algorithm".

Ein Nachteil an DQNs ist, dass der Raum der Aktionen diskret sein muss (also nur eine begrenzte Anzahl
an Werten erlaubt), da sich sonst keine Wertetabelle für die verschiedenen Aktionen errechnen lässt.

### (Soft) Actor Critic
Bei einem Actor Critic System hat man, ähnlich zu den GANs, zwei neuronale Netzwerke, die jedes ihr
eigenes Ziel verfolgen. Es gibt das "Actor"-Netzwerk, welches Entscheidungen über Aktionen trifft,
und ein separates "Critic"-Netzwerk, welches diese Entscheidungen bewertet. Die Bewertungen finden
dabei auch wieder in Form von Q-Werten statt.

Da das "Actor"-Netzwerk jetzt in der Lage ist, auch nicht diskrete Aktionen zu wählen, und das
"Critic"-Netzwerk diese bewertet ohne, dass wir eine Tabelle brauchen, kann man damit das Konzept
von DQNs auch auf Probleme mit komplexeren möglichen Aktionen anwenden. Hier wird dann statt einem
Gradientenabstieg ein Gradientenaufstieg verwendet, um die Q-Werte zu maximieren.