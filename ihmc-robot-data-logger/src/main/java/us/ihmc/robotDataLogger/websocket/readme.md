# Websocket data protocol

The static content (handshake and model) is served on a HTTP connection, while the data is send over websockets. When starting the logger, a static index.html is served on http://[hostname]:8008 with links to all the static content. This allows the user to download the model from the robot. Future functionality might include a way to record a few minutes of data to disk using the HTTP interface for customer support.

Binary websocket messages are used for variable data and change requests. Netty is used under the hood, pooling of buffers is used to have no recurring object allocation after the initial connection is setup. A few megabyte per connected client is expected, and almost impossible to avoid.

A simple command and echo server is available using text websocket messages (two messages types are defined in websockets, binary and text. This allows easy differentiation between the two without parsing the incoming data). The format is [COMMAND][Optional 5 digit argument].
Messages are parsed byte by byte to avoid string allocations.

Timestamps are published using raw UDP packets to sync the data to video if necessary.