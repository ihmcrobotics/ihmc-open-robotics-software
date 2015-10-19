# This message is emitted by the laser tilt controller when the laser hits
# one limit of its profile.
Header header

# signal == 0 => Half profile complete
# signal == 1 => Full Profile Complete
int32 signal
