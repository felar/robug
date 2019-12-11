# Tensorflow Agents
## Was ist Tensorflow?
Tensorflow ist eine Python-Bibliothek, die von Google entwickelt wurde, um
dem Umgang mit neuronalen Netzwerken zu vereinfachen. Ihr Ziel ist es, gleichzeitig
einen möglichst unkomplizierten Einstieg in das Thema für Anfänger zu bieten, aber
auch für fortgeschrittene Anwender interessant zu sein, indem es die Möglichkeit bietet,
eigene Aktivierungsfunktionen zu testen oder ein Modell auf mehreren GPUs zu trainieren.

Tensorflow is Open Source und wird ohne weitere Kosten von Google zur Verfügung gestellt.

## Was ist Tensorflow Agents?
Da wir allerdings nicht einfach ein "gewöhnliches" neuronales Netzwerk benötigen sondern
unseren Roboter mit Bestärkendem Lernen trainieren, reicht Tensorflow allein nicht aus.
Dafür gibt es eine Erweiterung namens "Tensorflow Agents", die auf der Grundlage von 
Tensorflow aufbaut, und sich spezifischer Bestärkendem Lernen widmet. Im Tensorflow
Agents Projekt finden sich viele Werkzeuge, die Bestärkendes Lernen einfacher machen,
sowie eine Auswahl an Architekturen, die bereits größtenteils implementiert sind, und
so "nurnoch" auf ein spezifisches Problem angewandt werden müssen.

Im weiteren Verlauf dieses Abschnitts wird es um einige der wichtigsten Konzepte innerhalb
von Tensorflow Agents gehen.

## Environments
Umgebungen sind der Teil, der am individuellsten auf das Problem angepasst werden muss.
Eine Umgebung in Tensorflow Agents definiert, in welchem Format der Agent seine Beobachtungen
erhält und was für mögliche Entscheidungen er treffen kann.

Im Code einer Umgebung müssen außerdem Methoden festgelegt werden, die es dem Agenten erlauben,
eine Entscheidung auszuführen, wenn er sie getroffen hat, sowie eine Möglichkeit, die Umgebung
zurückzusetzen, wenn eine Runde vorbei ist. Für jede getroffene Entscheidung gibt die Umgebung
außerdem die nächste Beobachtung und die Belohnung zurück, die der Agent erhalten hat.

## Policies
Strategien geben eine Zuordnung von einer Beobachtung zu einer Aktion an. Das kann, muss aber
kein neuronales Netzwerk sein. Wenn man denn so wollte, könnte man hierfür auch sein eigenes
kleines Programm schreiben, und es würde funktionieren. In der Regel handelt es sich hier aber
um ein neuronales Netzwerk von passender Architektur. Man beachte, dass die Strategie in diesem
Fall nur das Netzwerk beinhaltet - Training, Verlustfunktionen oder Optimierungsalgorithmen sind
hier nicht mit inbegriffen!

## Agents
Agenten sind schließĺich der Teil, an dem "das Denkende" zusammen kommt - hier wird die Verlustfunktion
und der Optimierungsalgorithmus zusammen mit der Strategie/dem Netzwerk angegeben. Der Agent kümmert sich
dann um das eigentliche Verfahren, das heißt das Training und Zusätze wie Dropout oder Learning Rate Decay.

## Drivers
Treiber sind nicht strikt nötig, um mit Tensorflow Agents zu arbeiten, sie vereinfachen es aber. Ein Treiber
ist in diesem Kontext nämlich dafür zuständig, die Daten anzusammeln. Das heißt man übergibt den Treiber
einfach die Anzahl an Episoden, die er pro Iteration machen soll, und ruft ihn dann zu einem geeigneten
Zeitpunkt auf. Der Treiber kümmert sich dann selbstständig darum, die entsprechende Anzahl an Episoden
zu durchlaufen und die gesammelte Erfahrung zu speichern. Üblicherweise passiert das mit einem _[Replay
Buffer](#replay-buffers)_. 
Gleichzeitig kann ein Treiber aber auch Statistiken wie Anzahl an Episoden und
durchschnittliche Belohnungen aufzeichnen.

## Replay Buffers
Replay Buffer funktionieren als einfacher Speicher, der sich auf das Format der Erfahrungen eines
neuronalen Netzwerks anpasst. Sie werden mit einer gewissen Größe initialisiert, und halten dann
ihnen gegebene Daten fest. Diese können dann als Ganzes wiedergegeben werden, oder (was für manche
Architekturen relevant ist) stichprobenartig eine Anzahl Erinnerungen noch einmal ausgeben.