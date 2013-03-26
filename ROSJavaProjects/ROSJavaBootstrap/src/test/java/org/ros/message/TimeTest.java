/*
 * Copyright (C) 2011 Google Inc.
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

package org.ros.message;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * @author kwc@willowgarage.com (Ken Conley)
 */
public class TimeTest {

  @Before
  public void setUp() {
  }

  @Test
  public void testConstructor() {
    // Test no args constructor.
    Time t = new Time();
    assertEquals(0, t.nsecs);
    assertEquals(0, t.secs);

    // Test secs/nsecs constructor with no normalization.
    t = new Time(1, 2);
    assertEquals(1, t.secs);
    assertEquals(2, t.nsecs);

    // Test secs/nsecs constructor with normalization.
    t = new Time(2, -1);
    assertEquals(1, t.secs);
    assertEquals(1000000000 - 1, t.nsecs);

    t = new Time(2, 1000000000 + 2);
    assertEquals(3, t.secs);
    assertEquals(2, t.nsecs);
  }

  @Test
  public void testFromMillis() {
    assertEquals(new Time(0, 0), Time.fromMillis(0));
    assertEquals(new Time(0, 1000000), Time.fromMillis(1));
    assertEquals(new Time(1, 0), Time.fromMillis(1000));
    assertEquals(new Time(10, 0), Time.fromMillis(10000));
    assertEquals(new Time(1, 1000000), Time.fromMillis(1001));
    assertEquals(new Time(1, 11000000), Time.fromMillis(1011));
  }

  @Test
  public void testNormalize() {
    Time t = new Time(0, 0);
    t.secs = 1;
    t.nsecs = 1000000000;
    t.normalize();
    assertEquals(2, t.secs);
    assertEquals(0, t.nsecs);

    t.secs = 1;
    t.nsecs = -1;
    t.normalize();
    assertEquals(0, t.secs);
    assertEquals(1000000000 - 1, t.nsecs);
  }

  @Test
  public void testIsZero() {
    assertTrue(new Time(0, 0).isZero());
    assertFalse(new Time(1, 0).isZero());
    assertFalse(new Time(0, 1).isZero());
  }

  @Test
  public void testComparable() {
    assertEquals(0, new Time(0, 0).compareTo(new Time(0, 0)));
    assertEquals(0, new Time(1, 1).compareTo(new Time(1, 1)));
    assertTrue(new Time(0, 1).compareTo(new Time(0, 0)) > 0);
    
    assertEquals(-1, new Time(0, 0).compareTo(new Time(0, 1)));
    assertTrue(new Time(0, 0).compareTo(new Time(0, 1)) < 0);
    assertTrue(new Time(1, 0).compareTo(new Time(0, 0)) > 0);
    assertTrue(new Time(0, 0).compareTo(new Time(1, 0)) < 0);

  }
}