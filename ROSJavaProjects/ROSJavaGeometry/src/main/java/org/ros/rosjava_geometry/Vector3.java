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

package org.ros.rosjava_geometry;

import com.google.common.collect.Lists;

import java.util.List;

/**
 * A three dimensional vector.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public class Vector3 {

  private static final Vector3 ZERO = new Vector3(0, 0, 0);
  private static final Vector3 X_AXIS = new Vector3(1, 0, 0);
  private static final Vector3 Y_AXIS = new Vector3(0, 1, 0);
  private static final Vector3 Z_AXIS = new Vector3(0, 0, 1);

  private final double x;
  private final double y;
  private final double z;

  public static Vector3 fromVector3Message(geometry_msgs.Vector3 message) {
    return new Vector3(message.getX(), message.getY(), message.getZ());
  }

  public static Vector3 fromPointMessage(geometry_msgs.Point message) {
    return new Vector3(message.getX(), message.getY(), message.getZ());
  }

  public static Vector3 zero() {
    return ZERO;
  }

  public static Vector3 xAxis() {
    return X_AXIS;
  }

  public static Vector3 yAxis() {
    return Y_AXIS;
  }

  public static Vector3 zAxis() {
    return Z_AXIS;
  }

  public Vector3(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public Vector3 add(Vector3 other) {
    return new Vector3(x + other.x, y + other.y, z + other.z);
  }

  public Vector3 subtract(Vector3 other) {
    return new Vector3(x - other.x, y - other.y, z - other.z);
  }

  public Vector3 invert() {
    return new Vector3(-x, -y, -z);
  }

  public double dotProduct(Vector3 other) {
    return x * other.x + y * other.y + z * other.z;
  }

  public Vector3 normalize() {
    return new Vector3(x / getMagnitude(), y / getMagnitude(), z / getMagnitude());
  }

  public Vector3 scale(double factor) {
    return new Vector3(x * factor, y * factor, z * factor);
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getZ() {
    return z;
  }

  public double getMagnitudeSquared() {
    return x * x + y * y + z * z;
  }

  public double getMagnitude() {
    return Math.sqrt(getMagnitudeSquared());
  }

  public geometry_msgs.Vector3 toVector3Message(geometry_msgs.Vector3 result) {
    result.setX(x);
    result.setY(y);
    result.setZ(z);
    return result;
  }

  public geometry_msgs.Point toPointMessage(geometry_msgs.Point result) {
    result.setX(x);
    result.setY(y);
    result.setZ(z);
    return result;
  }

  public boolean almostEquals(Vector3 other, double epsilon) {
    List<Double> epsilons = Lists.newArrayList();
    epsilons.add(x - other.x);
    epsilons.add(y - other.y);
    epsilons.add(z - other.z);
    for (double e : epsilons) {
      if (Math.abs(e) > epsilon) {
        return false;
      }
    }
    return true;
  }

  @Override
  public String toString() {
    return String.format("Vector3<x: %.4f, y: %.4f, z: %.4f>", x, y, z);
  }

  @Override
  public int hashCode() {
    // Ensure that -0 and 0 are considered equal.
    double x = this.x == 0 ? 0 : this.x;
    double y = this.y == 0 ? 0 : this.y;
    double z = this.z == 0 ? 0 : this.z;
    final int prime = 31;
    int result = 1;
    long temp;
    temp = Double.doubleToLongBits(x);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(y);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(z);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    Vector3 other = (Vector3) obj;
    // Ensure that -0 and 0 are considered equal.
    double x = this.x == 0 ? 0 : this.x;
    double y = this.y == 0 ? 0 : this.y;
    double z = this.z == 0 ? 0 : this.z;
    double otherX = other.x == 0 ? 0 : other.x;
    double otherY = other.y == 0 ? 0 : other.y;
    double otherZ = other.z == 0 ? 0 : other.z;
    if (Double.doubleToLongBits(x) != Double.doubleToLongBits(otherX))
      return false;
    if (Double.doubleToLongBits(y) != Double.doubleToLongBits(otherY))
      return false;
    if (Double.doubleToLongBits(z) != Double.doubleToLongBits(otherZ))
      return false;
    return true;
  }
}
