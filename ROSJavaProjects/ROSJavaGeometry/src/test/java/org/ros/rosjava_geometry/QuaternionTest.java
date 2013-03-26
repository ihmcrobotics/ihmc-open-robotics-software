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
public class QuaternionTest {

  @Test
  public void testAxisAngleToQuaternion() {
    Quaternion quaternion;

    quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), 0);
    assertEquals(0, quaternion.getX(), 1e-9);
    assertEquals(0, quaternion.getY(), 1e-9);
    assertEquals(0, quaternion.getZ(), 1e-9);
    assertEquals(1, quaternion.getW(), 1e-9);

    quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), Math.PI);
    assertEquals(0, quaternion.getX(), 1e-9);
    assertEquals(0, quaternion.getY(), 1e-9);
    assertEquals(1, quaternion.getZ(), 1e-9);
    assertEquals(0, quaternion.getW(), 1e-9);

    quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), Math.PI / 2);
    assertEquals(0, quaternion.getX(), 1e-9);
    assertEquals(0, quaternion.getY(), 1e-9);
    assertEquals(0.7071067811865475, quaternion.getZ(), 1e-9);
    assertEquals(0.7071067811865475, quaternion.getW(), 1e-9);

    quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), -Math.PI / 2);
    assertEquals(0, quaternion.getX(), 1e-9);
    assertEquals(0, quaternion.getY(), 1e-9);
    assertEquals(-0.7071067811865475, quaternion.getZ(), 1e-9);
    assertEquals(0.7071067811865475, quaternion.getW(), 1e-9);

    quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), 0.75 * Math.PI);
    assertEquals(0, quaternion.getX(), 1e-9);
    assertEquals(0, quaternion.getY(), 1e-9);
    assertEquals(0.9238795325112867, quaternion.getZ(), 1e-9);
    assertEquals(0.38268343236508984, quaternion.getW(), 1e-9);

    quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), -0.75 * Math.PI);
    assertEquals(0, quaternion.getX(), 1e-9);
    assertEquals(0, quaternion.getY(), 1e-9);
    assertEquals(-0.9238795325112867, quaternion.getZ(), 1e-9);
    assertEquals(0.38268343236508984, quaternion.getW(), 1e-9);

    quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), 1.5 * Math.PI);
    assertEquals(0, quaternion.getX(), 1e-9);
    assertEquals(0, quaternion.getY(), 1e-9);
    assertEquals(0.7071067811865475, quaternion.getZ(), 1e-9);
    assertEquals(-0.7071067811865475, quaternion.getW(), 1e-9);
  }

  @Test
  public void testInvert() {
    Quaternion inverse = Quaternion.fromAxisAngle(Vector3.zAxis(), Math.PI / 2).invert();
    assertEquals(0, inverse.getX(), 1e-9);
    assertEquals(0, inverse.getY(), 1e-9);
    assertEquals(-0.7071067811865475, inverse.getZ(), 1e-9);
    assertEquals(0.7071067811865475, inverse.getW(), 1e-9);
  }

  @Test
  public void testMultiply() {
    Quaternion quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), Math.PI / 2);
    Quaternion inverse = quaternion.invert();
    Quaternion rotated = quaternion.multiply(inverse);
    assertEquals(1, rotated.getW(), 1e-9);
  }

  @Test
  public void testRotateVector() {
    Quaternion quaternion = Quaternion.fromAxisAngle(Vector3.zAxis(), Math.PI / 2);
    Vector3 vector = new Vector3(1, 0, 0);
    Vector3 rotated = quaternion.rotateAndScaleVector(vector);
    assertEquals(0, rotated.getX(), 1e-9);
    assertEquals(1, rotated.getY(), 1e-9);
    assertEquals(0, rotated.getZ(), 1e-9);
  }
}
