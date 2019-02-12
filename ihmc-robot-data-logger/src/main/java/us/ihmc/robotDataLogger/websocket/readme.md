# Websocket data protocol

## Static content
The static content (handshake and model) is served on a HTTP connection, while the data is send over websockets. When starting the logger, a static index.html is served on http://\[hostname\]:8008 with links to all the static content. This allows the user to download the model from the robot. Future functionality might include a way to record a few minutes of data to disk using the HTTP interface for customer support.

## Variable data

Binary websocket messages are used for variable data and change requests. Netty is used under the hood, pooling of buffers is used to have no recurring object allocation after the initial connection is setup. A few megabyte per connected client is expected.


## Command and echo server
A simple command and echo server is available using text websocket messages (two messages types are defined in websockets, binary and text. This allows easy differentiation between the two without parsing the incoming data). The format is \[COMMAND\]\[Optional 5 digit argument\].
Messages are parsed byte by byte to avoid string allocations.


## Timestamps 
Timestamps are published using raw UDP packets to sync the data to video if necessary.

Timestamps are requested over the simple command server by sending SEND_TIMESTAMPSxxxxx, where xxxxx is the port number to send the timestamps to. 

Timestamps are send as 12 bytes messages containing a 4 byte header (UDPTimestampServer.TIMESTAMP_HEADER) and 8 byte timestamp (Java Long).

Timestamps are send from a special timestamp thread that polls frequently for new timestamps (at around 2kHz). There is a single publishing thread for all clients to avoid polling overhead as function of connected clients.  

## Autodiscovery
If a server is marked AutoDiscoverable in the DataServerSettings, it sends out multicast UDP messages on every interface on group 239.255.24.1 with port 55241.

The message is formated as JSON and the format is described in DataServerLocationBroadcast.PortPOJO.


## Keep alive

After a timeout (default 2500ms), the client will send the server a Websocket "Ping" packet. If the server does not respond after another timeout (by default, 5000ms in total), the server is considered "disconnected". This is to avoid hanging connection due to sudden disconnects of the network.


## FAQ
Note: the previous protocol was based on RTPS/DDS so some questions compare it to this implementation

Q: Will there be an allocation every time someone changes a YoVariable from the client side?
A: You get a single intermediate object allocation everytime a request comes in. Buffers get recycled (Netty has something like a recyclingarraylist that can grow, so if you send a lot it'll grow a bit but not forever). Given 24 bytes for the intermediate object. Let's say including overhead about 100 bytes/message in object allocation. Worst case will be a two-axis joystick input @ 100Hz. That's 20kbyte/second. So a bit over a megabyte/minute in allocations which I don't think poses a problem. If this becomes a problem, it won't be impossible to avoid but requires forking a bunch of classes (ugly and timeconsuming)

Q: Does the server allocate memory/Is the server realtime-safe.
A: About 10-20MB per connection. No cyclic allocations and no allocations when nothing connects. A small overhead receiving variable change requests. It is designed to be used in a realtime setting. 

Q: Is there a penalty per connection served by the server using websockets?  
A: Each connection uses the same amount of bandwidth. The Websocket solution could possible limit the amount of data send to specific clients.

