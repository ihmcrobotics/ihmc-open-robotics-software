/*
 * Copyright (C) 2012 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.math;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class RosMath {

  private RosMath() {
    // Utility class.
  }

  public static double clamp(double value, double minmum, double maximum) {
    if (value < minmum) {
      return minmum;
    }
    if (value > maximum) {
      return maximum;
    }
    return value;
  }

  public static float clamp(float value, float minmum, float maximum) {
    if (value < minmum) {
      return minmum;
    }
    if (value > maximum) {
      return maximum;
    }
    return value;
  }

  public static int clamp(int value, int minmum, int maximum) {
    if (value < minmum) {
      return minmum;
    }
    if (value > maximum) {
      return maximum;
    }
    return value;
  }

  public static long clamp(long value, long minmum, long maximum) {
    if (value < minmum) {
      return minmum;
    }
    if (value > maximum) {
      return maximum;
    }
    return value;
  }
}