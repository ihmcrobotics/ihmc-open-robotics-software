# Used to specify a transition in the SetMultiWaveform service.

float64 time # Transition time after start of period.
uint32 value # Value of the digital output after the transition time.
string topic # Topic to publish the transition timestamp to, or empty string if the transition should not be published.
