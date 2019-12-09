# Architekturen
Neuronales Netzwerk ist nicht gleich neuronales Netzwerk. Innerhalb dieses Themenbereiches gibt es viele
verschiedene Architekturen, die für verschiedene Problemstellungen gut funktionieren. Dazu gehört unter
anderem auch das Bestärkende Lernen, das wir in unserem Projekt verwenden. Aus diesem Grund hat diese
Architektur ein eigenes Kapitel.

## "Klassisches" Überwachtes Lernen
Hierbei handelt es sich um die einfachste Form von neuronalen Netzwerken, und entsprechend auch die,
die in Lektionen für Anfänger am häufigsten verwendet wird. Für diese Architektur hat man ein normales
neuronales Netzwerk und benutzt eine Verlustfunktion (in der Regel Kreuzentropie), die einfach den 
Unterschied zwischen dem Ergebnis des Netzwerks und dem "richtigen" Ergebnis, das man schon im Vorhinein
kennt, berechnet. Der Aufbau der Schichten ist dabei relativ frei und die Neuronen sind vollständig 
untereinander verbunden. In der Regel benutzt man für die letzte Schicht des Netzwerks `softmax` als
Aktivierungsfunktion, um eindeutigere Ergebnisse als Wahrscheinlichkeiten zu erhalten. Das ist aber
nur eine Konvention, und nicht immer sinnvoll (wenn die Ausgabe nicht als Wahrscheinlichkeit 
interpretiert wird).

Diese Art von Lernen ist am ehesten Vergleichbar mit dem Lernen in der Schule, 
wo vom Lehrer die korrekten Antworten vorgegeben werden. Es kann entsprechend nur für Probleme eingesetzt 
werden, in denen es klare Daten gibt, die ein richtiges Ergebnis vorgeben. Die häufigste Anwendung von 
dieser Architektur ist bei Klassifikationsproblemen, etwa beim erkennen von Handschrift oder Auswertung 
von Sensoren.

Der Begriff "Überwachtes Lernen" ist allerdings nicht exklusiv zu diesem Bereich, sondern wird auch als 
Kategorie für jede Architektur verwendet, bei dem es zu jeder Beobachtung klare Daten gibt, was die richtige
Antwort in diesem Fall ist.

## Unüberwachtes Lernen
Unüberwachtes Lernen ist für Fälle, in denen es keine Daten über die korrekte Antwort bei einer Beobachtung
gibt (oder geben kann). Der Aufbau der Schichten ist auch hier frei. Beim unüberwachten Lernen fehlen
allerdings die Daten für die richtige Antwort - und so kann keine einfache Verlustfuntkion verwendet werden,
die nur die Differenz zwischen zwei Vektoren berechnet. Stattdessen ist die Verlustfunktion hier stark
vom Problem abhängig, und muss spezifisch darauf angepasst werden. Sie repräsentiert die Bewertung, wie
gut eine Ausgabe war, ohne eine perfekte Antwort vorzugeben.

Die häufigsten Anwendungen dieser Architektur finden sich in Schätzproblemen, Clusteranalyse und Kompression
von Daten. Auch hier ist der Begriff "Unüberwachtes Lernen" eher ein Überbegriff für Architekturen, die ohne
vorgegebene Antworten auskommen - unter anderem gehört auch das Bestärkende Lernen dazu.

## Convolutional Neural Networks
Convolutional Neural Networks (oder auch **CNN**s, auf gut Deutsch "gefaltete neuronale Netzwerke") sind eine komplexere Verbesserung 
von Netzwerken zur Klassifikation. CNNs zeichnen sich dadurch aus, dass sie einen stark veränderten Aufbau 
der einzelnen Netzwerkschichten haben. In einem normalen NN besteht jede Schicht (inklusive der Eingabe) ja letztendlich 
nur aus einem großen Vektor. Dieses Vorgehen hat allerdings den Nachteil, dass dabei Informationen wie der räumliche 
Zusammenhang bei einem Bild oder die Farbe des Bildes verloren gehen. 
Das wird in CNNs geändert - hier wird schon die Eingabe in mehr Dimensionen angegeben. Pixel
bei einem Bild behalten dabei ihre X/Y-Koordinaten, und bestehen im Fall von Farbbildern sogar aus drei "Schichten" für
Rot, Grün und Blau. Das Resultat ist eine dreidimensionale Matrix, die jetzt wirklich alle Informationen aus dem Bild
enthält.

Ähnlich dreidimensional geht es auch mit den Schichten innerhalb des Netzwerks weiter. Wo wir in normalen NNs einfach
einen Vektor aus Neuronen hatten, wobei jedes Neuron mit jedem Eingabewert aus der vorigen Schicht verbunden war, sind
die Neuronen jetzt spezialisierter: Ein Neuron verarbeitet immer nur einen Ausschnitt der Eingabe, dessen Größe als
"Box" angegeben wird. Das Neuron daneben verarbeitet dann die gleiche Box um eins verschoben, und so weiter. Aus
diesem Schritt erhält man eine vorerst zweidimensionale Fläche aus Neuronen. Ein weiterer entscheidender Punkt ist dann
aber: Innerhalb dieser Schicht benutzen alle Neuronen die gleichen Gewichte! Das heißt, dass die komplette "Schicht"
nur auf ein Muster trainiert wird, dass sie irgendwo im Bild erkennen muss. Wenn man mehr Muster erkennen möchte, muss
man einen zweiten Satz an Gewichten verwenden, der genau wie beim ersten in einer zweidimensionalen Schicht aus Neuronen
resultiert. So erhalten wir letztendlich auch bei den Neuronen eine drei- (oder mehr) dimensionale Matrix aus Ergebnissen.

![Convolutional Neural Network Layer](https://raw.githubusercontent.com/felar/robug/master/pictures_gifs/convolutional_nns_visualization_by_martin_gorner.png)

Diese Art von Schichten ist deutlich effektiver zur Bilderkennung, wird aber immer noch mehr erweitert und verfeinert.
So gibt es inzwischen beispielsweise die "Inception" Architektur, die viele "gefaltete" Schichten hintereinander kombiniert,
aber sich auch Verzweigungen und Zusammenführungen der Schichten zu Nutze macht. 

## Rekurrente neuronale Netze
Rekurrente neuronale Netze (auch **RNN**s) ist eine Architektur von neuronalen Netzen, die sich speziell auf Probleme spezialisiert, die
stark von Kontext abhängig sind. Das häufigste Beispiel hierfür ist die Interpretation und Übersetzung von Sprache.

RNNs basieren auf der Idee, die Ergebnisse in den versteckten Schichten eines neuronalen
Netzwerks als Zustände zu verstehen, und diese Zustände weiter zu verwenden. Das heißt bei der Deutung von
beispielsweise dem Thema eines Satzes wird zuerst das erste Wort an das neuronale Netzwerk gegeben, welches eine
Deutung (also ein vermutetes Thema) generiert. Für das zweite Wort wird dann an das gleiche Netzwerk sowohl das Wort,
als auch der Zustand der versteckten Schichten vom ersten Wort übergeben - die Werte aus den versteckten Schichten
werden einfach an den Vektor des Wortes angehängt. So geht die Prozedur dann weiter, bis das Netzwerk am Ende des Satzes
eine Entscheidung getroffen hat, welches Thema der Satz behandelt.

Für die Architektur der versteckten Schichten gibt es bei RNNs verschiedene Ansätze, die
über die Jahre entwickelt wurden, da das Netzwerk bei einer simplen Architektur (wie hier beschrieben) Schwierigkeiten
hat, verlässlich auf einen optimalen Punkt zuzulaufen (und dort schließlich stehen zu bleiben). Die wichtigste
Eigenschaft dieser Architekturen sind die sogenannten "Forget Gates", die die Lang- und Kurzzeiterinnerung des Netzwerks
regulieren. Beispiele für diese Architekturen (auch "Zellen" genannt) sind GRU Zellen (Generalized Recurrent Unit) oder
LSTM zellen (Long Short Term Memory).

Worauf man bei diesem Beispiel achten muss, ist dass ein RNN allein hier nicht ausreicht: Wörter
müssen natürlich überhaupt einmal als Vektoren repräsentiert werden. Das passiert in der Regel über vorher (ebenfalls
über neuronale Netzwerke trainierte) Matrizen, die jedem Wort einer Sprache einen Wert zuordnen. Da diese Matrizen mit
großen Mengen an Text trainiert werden und eine "Landschaft" aus Wörtern ergeben, hat das Resultat nützliche Eigenschaften
wie dass (themen-)verwandte Wörter Werte zugewiesen werden, die nahe beieinander liegen. So lassen sich solche Listen
häufig bei anderen Anwendungen wiederverwendet werden, oder können als Basis genutzt werden, die nurnoch kleine
Modifikationen benötigt, um auf das neue Problem angewendet werden zu können.
Weitere Methoden die verwendet werden, um Sprache besser interpretieren zu können sind beispielsweise ein zusätzliches
Netzwerk, dass jedem Wort in einem Satz Gewichtungen zuweist, so dass Dinge wie Artikel nicht so schwer ins Gewicht
fallen. Außerdem werden rekurrente Netzwerke ein Satz einmal vorwärts und einmal rückwärts gegeben, damit auch Interpretationen
möglich sind, bei denen Wörter am Anfang des Satzes mehr Bedeutung bekommen durch ein Wort gegen Ende.

## Generative Adversarial Networks
Generative Adversarial Networks, kurz **GAN**s, sind keine Architektur, so wie die vorigen Varianten waren, sondern
eine Kombination aus neuronalen Netzwerken, die sich besonders gut für eine Sorte Probleme eignet: Das Generieren von
Daten (häufig sind das Bilder, das muss aber nicht sein). Bei GANs handelt es sich um eine Form des Unüberwachten Lernens.

Die Vorgehensweise von GANs ist im Grunde nicht kompliziert: Man lässt zwei separate neuronale Netzwerke gegeneinander
antreten. Das eine Netzwerk hat die Aufgabe, möglichst echt aussehende Daten zu generieren - die Eingabe ist hier nicht
so wichtig wie bei anderen Netzwerken - sie kann zufällig sein, manchmal auch so etwas wie eine von Hand gemachte Skizze.
Dieses Netzwerk nennt man allgemein den Generator.

Das zweite Netzwerk hat dann aber die Aufgabe, echte Daten von generierten Daten zu unterscheiden. Dazu erhält es einen
Datensatz, der teils aus echten Daten (beispielsweise Katzenbildern) besteht, und teils aus Daten, die vom Generator
generiert wurden. Es wird dann darauf trainiert, die beiden Sorten möglichst gut voneinander unterscheiden zu können.
Dieses Netzwerk nennt man den Diskriminator.

Die Genauigkeit, mit der der Diskriminator echte und gefälschte Daten auseinander halten kann, wird dann anstatt einer
normalen Verlustfunktion als Verlust für den Generator verwendet. Dieser wird so auf Dauer gezwungen, immer perfektere
Daten zu erzeugen. So soll er (zumindest in der Theorie) über genug Iterationen schlussendlich lernen, perfekte Imitationen
der realen Daten zu erzeugen.

In der Praxis sind die Resultate leider oft nicht ganz so gut, wie erhofft. Auch muss man sehr auf die Balance zwischen
den beiden Netzwerken achten: Wenn eins der beiden sehr viel schlechter oder sehr viel besser als das andere abschneidet,
bricht schnell das ganze System zusammen, weil es für den Gegenspieler kaum noch möglich ist, die "richtige Richtung"
zu finden und somit effektiv zu lernen. Auch sind die Ergebnisse nicht selten für den Diskriminator nicht unterscheidbar
von echten Bildern, sehen für Menschen aber besonders bei näherer Betrachtung noch verzerrt aus.

![Pix2Pix Cat Example](https://raw.githubusercontent.com/felar/robug/master/pictures_gifs/pix2pix_cat_example.png)

_Man kann dieses vor-trainierte Netzwerk auch [hier](https://affinelayer.com/pixsrv/) selbst im Browser ausprobieren.
In diesem Beispiel wurde als Eingabe für den Generator natürlich die Silhouette verwendet und keine zufälligen Werte._

