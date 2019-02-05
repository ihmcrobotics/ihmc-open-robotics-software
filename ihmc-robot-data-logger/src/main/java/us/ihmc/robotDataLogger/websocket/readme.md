# Websocket data protocol

The static content (handshake and model) is served on a HTTP connection, while the data is send over websockets. When starting the logger, a static index.html is served on http://\[hostname\]:8008 with links to all the static content. This allows the user to download the model from the robot. Future functionality might include a way to record a few minutes of data to disk using the HTTP interface for customer support.

Binary websocket messages are used for variable data and change requests. Netty is used under the hood, pooling of buffers is used to have no recurring object allocation after the initial connection is setup. A few megabyte per connected client is expected, and almost impossible to avoid.

A simple command and echo server is available using text websocket messages (two messages types are defined in websockets, binary and text. This allows easy differentiation between the two without parsing the incoming data). The format is \[COMMAND\]\[Optional 5 digit argument\].
Messages are parsed byte by byte to avoid string allocations.

Timestamps are published using raw UDP packets to sync the data to video if necessary.


## FAQ
Note: the previous protocol was based on RTPS/DDS so some questions compare it to this implementation

Q: Is it just as safe to publish a websocket message from the controller as it is an RTPS message? With RTPS it happens in the "realtime" rtps thread. Is there anything special required to make the UDP timestamp publish safe?

A: Actually, I looked at the code and the RTPS timestamp publish is in a separate thread because it posed problems. Before the switch to RTPS, we used to publish UDP messages on the realtime thread without issues. Steppr/wanderer did all their control messages using UDP so I don't think it'll pose a problem. Nothing special, UDP doesn't really block by design. Messages just clog up the network if you send too much.

Q: Will there be an allocation every time someone changes a YoVariable from the client side?
A: You get a single intermediate object allocation everytime a request comes in. Buffers get recycled (Netty has something like a recyclingarraylist that can grow, so if you send a lot it'll grow a bit but not forever). Given 24 bytes for the intermediate object. Let's say including overhead about 100 bytes/message in object allocation. Worst case will be a two-axis joystick input @ 100Hz. That's 20kbyte/second. So a bit over a megabyte/minute in allocations which I don't think poses a problem. If this becomes a problem, it won't be impossible to avoid but requires forking a bunch of classes (ugly and timeconsuming)

Q: Does the server allocate memory/Is the server realtime-safe.
A: About 10-20MB per connection. No cyclic allocations and no allocations when nothing connects. A small overhead receiving variable change requests. It is designed to be used in a realtime setting. 

Q: Is there a penalty per connection served by the server using websockets?  
A: Each connection uses the same amount of bandwidth. The Websocket solution could possible limit the amount of data send to specific clients.

