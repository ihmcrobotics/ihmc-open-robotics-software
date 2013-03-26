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

import com.google.common.annotations.VisibleForTesting;

import org.ros.message.Time;
import org.ros.namespace.GraphName;

/**
 * A transformation in terms of translation, rotation, and scale.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public class Transform {

  private Vector3 translation;
  private Quaternion rotationAndScale;

  public static Transform fromTransformMessage(geometry_msgs.Transform message) {
    return new Transform(Vector3.fromVector3Message(message.getTranslation()),
        Quaternion.fromQuaternionMessage(message.getRotation()));
  }

  public static Transform fromPoseMessage(geometry_msgs.Pose message) {
    return new Transform(Vector3.fromPointMessage(message.getPosition()),
        Quaternion.fromQuaternionMessage(message.getOrientation()));
  }

  public static Transform identity() {
    return new Transform(Vector3.zero(), Quaternion.identity());
  }

  public static Transform xRotation(double angle) {
    return new Transform(Vector3.zero(), Quaternion.fromAxisAngle(Vector3.xAxis(), angle));
  }

  public static Transform yRotation(double angle) {
    return new Transform(Vector3.zero(), Quaternion.fromAxisAngle(Vector3.yAxis(), angle));
  }

  public static Transform zRotation(double angle) {
    return new Transform(Vector3.zero(), Quaternion.fromAxisAngle(Vector3.zAxis(), angle));
  }

  public static Transform translation(double x, double y, double z) {
    return new Transform(new Vector3(x, y, z), Quaternion.identity());
  }

  public static Transform translation(Vector3 vector) {
    return new Transform(vector, Quaternion.identity());
  }

  public Transform(Vector3 translation, Quaternion rotation) {
    this.translation = translation;
    this.rotationAndScale = rotation;
  }

  /**
   * Apply another {@link Transform} to this {@link Transform}.
   * 
   * @param other
   *          the {@link Transform} to apply to this {@link Transform}
   * @return the resulting {@link Transform}
   */
  public Transform multiply(Transform other) {
    return new Transform(apply(other.translation), apply(other.rotationAndScale));
  }

  public Transform invert() {
    Quaternion inverseRotationAndScale = rotationAndScale.invert();
    return new Transform(inverseRotationAndScale.rotateAndScaleVector(translation.invert()),
        inverseRotationAndScale);
  }

  public Vector3 apply(Vector3 vector) {
    return rotationAndScale.rotateAndScaleVector(vector).add(translation);
  }

  public Quaternion apply(Quaternion quaternion) {
    return rotationAndScale.multiply(quaternion);
  }

  public Transform scale(double factor) {
    return new Transform(translation, rotationAndScale.scale(Math.sqrt(factor)));
  }

  public double getScale() {
    return rotationAndScale.getMagnitudeSquared();
  }

  /**
   * @see <a
   *      href="http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion">Quaternion
   *      rotation matrix</a>
   */
  public double[] toMatrix() {
    double x = rotationAndScale.getX();
    double y = rotationAndScale.getY();
    double z = rotationAndScale.getZ();
    double w = rotationAndScale.getW();
    double mm = rotationAndScale.getMagnitudeSquared();
    return new double[] { mm - 2 * y * y - 2 * z * z, 2 * x * y + 2 * z * w, 2 * x * z - 2 * y * w,
        0, 2 * x * y - 2 * z * w, mm - 2 * x * x - 2 * z * z, 2 * y * z + 2 * x * w, 0,
        2 * x * z + 2 * y * w, 2 * y * z - 2 * x * w, mm - 2 * x * x - 2 * y * y, 0,
        translation.getX(), translation.getY(), translation.getZ(), 1 };
  }

  public geometry_msgs.Transform toTransformMessage(geometry_msgs.Transform result) {
    result.setTranslation(translation.toVector3Message(result.getTranslation()));
    result.setRotation(rotationAndScale.toQuaternionMessage(result.getRotation()));
    return result;
  }

  public geometry_msgs.Pose toPoseMessage(geometry_msgs.Pose result) {
    result.setPosition(translation.toPointMessage(result.getPosition()));
    result.setOrientation(rotationAndScale.toQuaternionMessage(result.getOrientation()));
    return result;
  }

  public geometry_msgs.PoseStamped toPoseStampedMessage(GraphName frame, Time stamp,
      geometry_msgs.PoseStamped result) {
    result.getHeader().setFrameId(frame.toString());
    result.getHeader().setStamp(stamp);
    result.setPose(toPoseMessage(result.getPose()));
    return result;
  }

  public boolean almostEquals(Transform other, double epsilon) {
    return translation.almostEquals(other.translation, epsilon)
        && rotationAndScale.almostEquals(other.rotationAndScale, epsilon);
  }

  @VisibleForTesting
  Vector3 getTranslation() {
    return translation;
  }

  @VisibleForTesting
  Quaternion getRotationAndScale() {
    return rotationAndScale;
  }

  @Override
  public String toString() {
    return String.format("Transform<%s, %s>", translation, rotationAndScale);
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((rotationAndScale == null) ? 0 : rotationAndScale.hashCode());
    result = prime * result + ((translation == null) ? 0 : translation.hashCode());
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
    Transform other = (Transform) obj;
    if (rotationAndScale == null) {
      if (other.rotationAndScale != null)
        return false;
    } else if (!rotationAndScale.equals(other.rotationAndScale))
      return false;
    if (translation == null) {
      if (other.translation != null)
        return false;
    } else if (!translation.equals(other.translation))
      return false;
    return true;
  }
}
