# Transitions will occur at k * period + zero_offset + transitions[j].time, where j and
# k are integers.

float64 period # Period of the waveform in seconds.
float64 zero_offset # Time corresponding to a time of 0 in times[] in seconds
MultiWaveformTransition[] transitions # Transitions in the waveform. Transition times should be in increasing order, and be between 0 (inclusive) and period (exclusive)
