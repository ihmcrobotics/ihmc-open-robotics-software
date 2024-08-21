package org.ros.time;

import org.ros.message.Time;

public interface TimeProvider {
   Time getCurrentTime();
}