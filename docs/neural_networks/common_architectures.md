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
Convolutional Neural Networks (oder auch CNNs, auf gut Deutsch "gefaltete neuronale Netzwerke") sind eine komplexere Verbesserung 
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

## Recurrent


## Architecture Search