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

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class UnsignedTest {

  @Test
  public void testIntToLong() {
    assertEquals(Unsigned.intToLong(0xffffffff), 0xffffffffl);
    assertEquals(Unsigned.intToLong(0xffff), 0xffffl);
    assertEquals(Unsigned.intToLong(0), 0);
    assertEquals(Unsigned.intToLong(42), 42);
  }

  @Test
  public void testShortToInt() {
    assertEquals(Unsigned.shortToInt((short) 0xffff), 0xffff);
    assertEquals(Unsigned.shortToInt((short) 0xff), 0xff);
    assertEquals(Unsigned.shortToInt((short) 0), 0);
    assertEquals(Unsigned.shortToInt((short) 42), 42);
  }

  @Test
  public void testByteToShort() {
    assertEquals(Unsigned.byteToShort((byte) 0xff), 0xff);
    assertEquals(Unsigned.byteToShort((byte) 0xf), 0xf);
    assertEquals(Unsigned.byteToShort((byte) 0), 0);
    assertEquals(Unsigned.byteToShort((byte) 42), 42);
  }
}
