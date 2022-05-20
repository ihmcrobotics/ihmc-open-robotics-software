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
 * Provides conversions from unsigned values to bitwise equal signed values.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class Unsigned {

  private Unsigned() {
    // Utility class.
  }

  /**
   * @param value
   *          an unsigned {@link Integer} value
   * @return a signed {@link Long} that is bitwise equal to {@code value}
   */
  public static long intToLong(int value) {
    return value & 0xffffffffl;
  }

  /**
   * @param value
   *          an unsigned {@link Short} value
   * @return a signed {@link Integer} that is bitwise equal to {@code value}
   */
  public static int shortToInt(short value) {
    return value & 0xffff;
  }

  /**
   * @param value
   *          an unsigned {@link Byte} value
   * @return a signed {@link Short} that is bitwise equal to {@code value}
   */
  public static short byteToShort(byte value) {
    return (short) (value & 0xff);
  }
}
