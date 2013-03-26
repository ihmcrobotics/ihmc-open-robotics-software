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
public class DurationTest {

  @Before
  public void setUp()  {
  }

  @Test
  public void testConstructor() {
    // Test no args constructor.
    Duration t = new Duration();
    assertEquals(0, t.nsecs);
    assertEquals(0, t.secs);
      
    // Test secs/nsecs constructor with no normalization.
    t = new Duration(1, 2);
    assertEquals(1, t.secs);
    assertEquals(2, t.nsecs);

    // Test secs/nsecs constructor with normalization.
    t = new Duration(2, -1);
    assertEquals(1, t.secs);
    assertEquals(1000000000 - 1, t.nsecs);
    
    t = new Duration(2, 1000000000 + 2);
    assertEquals(3, t.secs);
    assertEquals(2, t.nsecs);
  }
  
  @Test
  public void testNormalize() { 
    Duration d = new Duration(0, 0);
    d.secs = 1;
    d.nsecs = 1000000000;
    d.normalize();
    assertEquals(2, d.secs);
    assertEquals(0, d.nsecs);
    
    d.secs = 1;
    d.nsecs = -1;
    d.normalize();
    assertEquals(0, d.secs);
    assertEquals(1000000000-1, d.nsecs);
  }
  
  @Test
  public void testIsZero() {
    assertTrue(new Duration(0, 0).isZero());
    assertFalse(new Duration(1, 0).isZero());
    assertFalse(new Duration(0, 1).isZero());
  }
  
  @Test
  public void testComparable() {
    assertEquals(0, new Duration(0, 0).compareTo(new Duration(0, 0)));
    assertEquals(0, new Duration(1, 0).compareTo(new Duration(1, 0)));
    
    assertTrue(new Duration(0, 0).compareTo(new Duration(0, -1)) > 0);
    assertTrue(new Duration(0, -1).compareTo(new Duration(0, 0)) < 0);
    
    assertTrue(new Duration(0, 0).compareTo(new Duration(-1, 0)) > 0);
    assertTrue(new Duration(-1, 0).compareTo(new Duration(0, 0)) < 0);
    
    assertTrue(new Duration(1, 0).compareTo(new Duration(0, 0)) > 0);
    assertTrue(new Duration(0, 0).compareTo(new Duration(1, 0)) < 0);
    
    assertTrue(new Duration(0, 1).compareTo(new Duration(0, 0)) > 0);
    assertTrue(new Duration(0, 0).compareTo(new Duration(0, 1)) < 0);
  }
}