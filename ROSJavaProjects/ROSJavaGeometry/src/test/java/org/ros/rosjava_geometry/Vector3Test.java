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

package org.ros.rosjava_geometry;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class Vector3Test {

  @Test
  public void testAdd() {
    Vector3 vector1 = new Vector3(1, 2, 3);
    Vector3 vector2 = new Vector3(2, 3, 4);
    Vector3 result = vector1.add(vector2);
    assertEquals(result.getX(), 3, 1e-9);
    assertEquals(result.getY(), 5, 1e-9);
    assertEquals(result.getZ(), 7, 1e-9);
  }

  @Test
  public void testSubtract() {
    Vector3 vector1 = new Vector3(1, 2, 3);
    Vector3 vector2 = new Vector3(2, 3, 4);
    Vector3 result = vector1.subtract(vector2);
    assertEquals(result.getX(), -1, 1e-9);
    assertEquals(result.getY(), -1, 1e-9);
    assertEquals(result.getZ(), -1, 1e-9);
  }

  @Test
  public void testInvert() {
    Vector3 result = new Vector3(1, 1, 1).invert();
    assertEquals(result.getX(), -1, 1e-9);
    assertEquals(result.getY(), -1, 1e-9);
    assertEquals(result.getZ(), -1, 1e-9);
  }

  @Test
  public void testDotProduct() {
    Vector3 vector1 = new Vector3(1, 2, 3);
    Vector3 vector2 = new Vector3(2, 3, 4);
    assertEquals(20.0, vector1.dotProduct(vector2), 1e-9);
  }

  @Test
  public void testLength() {
    assertEquals(2, new Vector3(2, 0, 0).getMagnitude(), 1e-9);
    assertEquals(2, new Vector3(0, 2, 0).getMagnitude(), 1e-9);
    assertEquals(2, new Vector3(0, 0, 2).getMagnitude(), 1e-9);
    assertEquals(Math.sqrt(3), new Vector3(1, 1, 1).getMagnitude(), 1e-9);
  }
}
