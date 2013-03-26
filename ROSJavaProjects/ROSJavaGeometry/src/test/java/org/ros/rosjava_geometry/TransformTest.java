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
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TransformTest {

  @Test
  public void testMultiply() {
    Transform transform1 = new Transform(Vector3.xAxis(), Quaternion.identity());
    Transform transform2 =
        new Transform(Vector3.yAxis(), Quaternion.fromAxisAngle(Vector3.zAxis(), Math.PI / 2));

    Transform result1 = transform1.multiply(transform2);
    assertEquals(1.0, result1.getTranslation().getX(), 1e-9);
    assertEquals(1.0, result1.getTranslation().getY(), 1e-9);
    assertEquals(0.0, result1.getTranslation().getZ(), 1e-9);
    assertEquals(0.0, result1.getRotationAndScale().getX(), 1e-9);
    assertEquals(0.0, result1.getRotationAndScale().getY(), 1e-9);
    assertEquals(0.7071067811865475, result1.getRotationAndScale().getZ(), 1e-9);
    assertEquals(0.7071067811865475, result1.getRotationAndScale().getW(), 1e-9);

    Transform result2 = transform2.multiply(transform1);
    assertEquals(0.0, result2.getTranslation().getX(), 1e-9);
    assertEquals(2.0, result2.getTranslation().getY(), 1e-9);
    assertEquals(0.0, result2.getTranslation().getZ(), 1e-9);
    assertEquals(0.0, result2.getRotationAndScale().getX(), 1e-9);
    assertEquals(0.0, result2.getRotationAndScale().getY(), 1e-9);
    assertEquals(0.7071067811865475, result2.getRotationAndScale().getZ(), 1e-9);
    assertEquals(0.7071067811865475, result2.getRotationAndScale().getW(), 1e-9);
  }

  @Test
  public void testInvert() {
    Transform transform =
        new Transform(Vector3.yAxis(), Quaternion.fromAxisAngle(Vector3.zAxis(), Math.PI / 2));
    Transform inverse = transform.invert();

    assertEquals(-1.0, inverse.getTranslation().getX(), 1e-9);
    assertEquals(0.0, inverse.getTranslation().getY(), 1e-9);
    assertEquals(0.0, inverse.getTranslation().getZ(), 1e-9);
    assertEquals(0.0, inverse.getRotationAndScale().getX(), 1e-9);
    assertEquals(0.0, inverse.getRotationAndScale().getY(), 1e-9);
    assertEquals(-0.7071067811865475, inverse.getRotationAndScale().getZ(), 1e-9);
    assertEquals(0.7071067811865475, inverse.getRotationAndScale().getW(), 1e-9);

    Transform neutral = transform.multiply(inverse);
    assertTrue(neutral.almostEquals(Transform.identity(), 1e-9));
  }

  @Test
  public void testInvertRandom() {
    Random random = new Random();
    random.setSeed(42);
    for (int i = 0; i < 10000; i++) {
      Vector3 vector = randomVector(random);
      Quaternion quaternion = randomQuaternion(random);
      Transform transform = new Transform(vector, quaternion);
      Transform inverse = transform.invert();
      Transform neutral = transform.multiply(inverse);
      assertTrue(neutral.almostEquals(Transform.identity(), 1e-9));
    }
  }

  @Test
  public void testMultiplyRandom() {
    Random random = new Random();
    random.setSeed(42);
    for (int i = 0; i < 10000; i++) {
      Vector3 data = randomVector(random);
      Vector3 vector1 = randomVector(random);
      Vector3 vector2 = randomVector(random);
      Quaternion quaternion1 = randomQuaternion(random);
      Quaternion quaternion2 = randomQuaternion(random);
      Transform transform1 = new Transform(vector1, quaternion1);
      Transform transform2 = new Transform(vector2, quaternion2);
      Vector3 result1 = transform1.apply(transform2.apply(data));
      Vector3 result2 = transform1.multiply(transform2).apply(data);
      assertTrue(result1.almostEquals(result2, 1e-9));
    }
  }

  @Test
  public void testScale() {
    assertTrue(Vector3.xAxis().scale(2)
        .almostEquals(Transform.identity().scale(2).apply(Vector3.xAxis()), 1e-9));
  }

  private Quaternion randomQuaternion(Random random) {
    return new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(),
        random.nextDouble());
  }

  private Vector3 randomVector(Random random) {
    return new Vector3(random.nextDouble(), random.nextDouble(), random.nextDouble());
  }
}
