
# Graph Concepts
ROS2 ist eine Middleware, das heißt eine Software für die Kommunikation zwischen verschiedenen Programmen und Programmiersprachen.

Es besteht zum Großteil aus dem ROS Graphen, einem Netzwerk aus Nodes und Verbindungen zwischen diesen.

## Nodes
Nodes sind Entitäten, die miteinander kommunizieren können, indem sie zu Topics subcriben und Messages publishen.
Sie können außerdem Parameterwerte und Funktionen enthalten oder Funktionen anderer Nodes benutzen.

## Messages
Messages sind ein ROS-interner Datentyp zu publishen und subscriben. 
Sie stellen sozusagen die Nachricht dar, die weitergegeben werden soll.

## Topics
Topics sind so etwas wie die schwarzen Bretter, an die die Messages gepinnt werden können. 
Nodes können diese subscriben um die Messages zu empfangen oder an sie etwas publishen.
Topics haben einen bestimmten Message-Datentyp, den sie 'empfangen' können.

### Publishen und Subscriben
Beim Publishen wird eine Message von einer Node an ein Topic gegeben und dort gespeichert. 
Diese Message kann durch das Subscriben an andere Nodes weitergegeben werden. 
Wenn eine Node zu einem Topic subscribed, erhält sie alle Messages, die an dieses Topic gepublished werden.
Es können beliebig viele Nodes etwas an ein Topic gepublished werden und es können beliebig viele Nodes ein Topic subscriben.

## Services und Clients
Services sind sozusagen Nodes, die von einem Programm (beispielsweise der Simulation) gestartet
werden und von anderen Nodes aufgerufen werden können. Dann führen sie ihre zugehörige Aktion
aus. Damit man von seiner eigenen Node einen Service aufrufen kann, muss man einen sogenannten
Client erstellen, der für diesen Service zuständig ist. Über den Client kann der Service
dann aufgerufen werden.

## Discovery 
Die Discovery ist der Prozess in dem die Nodes entscheiden ob und wie sie miteinander kommunizieren. 
Dies wird von ROS2 automatisch ausgeführt. 
Im Groben sorgt die Discovery dafür, dass die Nodes den anderen Nodes melden, wenn sie im Netzwerk online oder offline gehen.