# Verbesserung der Leistung von neuronalen Netzwerken

Über die Wahl von richtigen Hyperparametern hinaus gibt es noch weitere
Techniken, mit denen man die Leistung eines neuronalen Netzwerks verbessern kann.
Hier sind einige der populärsten Methoden dafür:

## Learning Rate Decay
Bei Learning Rate Decay handelt es sich um genau das, was der Begriff beschreibt:
Die Lernrate verfällt, je länger das Training geht. Das heißt, dass am Anfang des
Trainings eine relativ hohe Lernrate genutzt wird, die dann aber im Laufe der Iterationen
immer weiter sinkt. 

Das bewirkt, dass das Netzwerk am Anfang relativ große "Schritte" machen kann, um schneller
eine grobe funktionierende Taktik zu erhalten. Dadurch, dass die Lernrate danach kleiner wird,
kann das Netzwerk sich dann in immer kleineren Details verbessern. Einige [Optimierungsalgorithmen](optimizer.md)
nutzen Learning Rate Decay (oder etwas vergleichbares) bereits von allein, andere muss man dafür
anpassen.

Der Effekt von Learning Rate Decay lässt sich am besten feststellen anhand des Verlaufs des
Verlusts: Ohne LRD ist diese auch nach langem Trainieren immernoch nicht sehr stabil, und bricht häufig
nach oben und unten aus. Das ist ein Resultat davon, dass die Schritte des Opimierungsalgorithmus
nicht groß genug sind. Mit LRD wird dieser Grapp deutlich stabiler.

## Dropout
In vielen neuronalen Netzwerken entwickelt sich beim längerem Training ein Problem
namens "Overfitting". Das bedeutet, dass das Netzwerk sich zwar gut bei den Trainingsdaten
schlägt, aber zunehmend schlechte Ergebnisse bei Testdaten oder einem realen Szenario erzielt.

Dieses Problem kommt in der Regel davon, dass das Netzwerk sich zu präzise auf die Trainingsdaten
anpasst. Wenn es sich zu sehr auf genau diese Daten spezialisiert, leidet darunter dann die
Übertragbarkeit des Netzwerks auf andere Szenarien. Ursachen dafür können sein, dass man nicht
genug unterschiedliche Trainingsdaten hat, oder dass man zu viele Neuronen und Schichten verwendet
in einem Extremfall kann das Netzwerk dann nämlich einfach die Trainingsdaten mit richtigen
Antworten auswendig lernen - was ja nicht Ziel des Ganzen ist.

Eine populäre Lösung für Overfitting ist es, Gewalt anzuwenden - um genauer zu sein, man schiesst auf
die Neuronen. Das heißt, man spezifiziert in einer Iteration eine Wahrscheinlichkeit, mit der "auf ein
Neuron geschossen wird". Das könnte zum Beispiel heißen, dass 25% der Neuronen für diese Iteration zum
Dropout ausgewählt werden. Das heißt dann, dass die ausgewählten Neuronen in dieser Iteration
"eingefroren" sind - sie werden nicht vom Optimierungsalgorithmus verändert. In der nächsten Iteration
werden die Neuronen, die ausgeschlossen werden, dann wieder neu ausgewürfelt.

Mit dieser Methode werden so immer nur kleine (zufällige) Teile des Netzwerks gleichzeitig verändert. Damit werden
die Freiheitsgrade, die das Netzwerk hat, drastisch reduziert. Dadurch ist das Netzwerk gezwungen, Kategorien
für die Daten zu lernen (zumindest im Fall von Klassifizierungsproblemen), statt eine zu individuell auf die
Trainingsdaten angepasste Taktik zu entwickeln.

## Weitere
Es gibt natürlich noch weitere Methoden, manche davon auch spezifisch für gewisse Architekturen (einige, die
hauptsächlich auf bestärkendes Lernen zutreffen, sind auch in [Bestärkendes Lernen](reinforcement_learning.md) 
dargestellt). Eine sehr populäre Möglichkeit ist beispielsweise _Batch Normalization_, bei der die Gewichte
und Schwellenwerte normalisiert werden, bevor sie durch die Aktivierungsfunktion gehen. Solche und ähnliche
Methoden sind auch ein beträchtlicher Teil der Forschung an neuronalen Netzwerken.