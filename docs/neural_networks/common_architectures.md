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

## Convolutional

## Recurrent


## Architecture Search