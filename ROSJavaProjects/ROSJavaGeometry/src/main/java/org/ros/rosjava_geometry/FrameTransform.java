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

import com.google.common.base.Preconditions;

import org.ros.message.Time;
import org.ros.namespace.GraphName;

/**
 * Describes a {@link Transform} from data in the source frame to data in the
 * target frame at a specified {@link Time}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class FrameTransform {

  private final Transform transform;
  private final GraphName source;
  private final GraphName target;
  private final Time time;

  public static FrameTransform fromTransformStampedMessage(
      geometry_msgs.TransformStamped transformStamped) {
    Transform transform = Transform.fromTransformMessage(transformStamped.getTransform());
    String target = transformStamped.getHeader().getFrameId();
    String source = transformStamped.getChildFrameId();
    Time stamp = transformStamped.getHeader().getStamp();
    return new FrameTransform(transform, GraphName.of(source), GraphName.of(target), stamp);
  }

  /**
   * Allocates a new {@link FrameTransform}.
   * 
   * @param transform
   *          the {@link Transform} that transforms data in the {@code source}
   *          frame to data in the {@code target} frame
   * @param source
   *          the source frame
   * @param target
   *          the target frame
   * @param time
   *          the time associated with this {@link FrameTransform}, can be
   *          {@null}
   */
  public FrameTransform(Transform transform, GraphName source, GraphName target, Time time) {
    Preconditions.checkNotNull(transform);
    Preconditions.checkNotNull(source);
    Preconditions.checkNotNull(target);
    this.transform = transform;
    this.source = source;
    this.target = target;
    this.time = time;
  }

  public Transform getTransform() {
    return transform;
  }

  public GraphName getSourceFrame() {
    return source;
  }

  public GraphName getTargetFrame() {
    return target;
  }

  public FrameTransform invert() {
    return new FrameTransform(transform.invert(), target, source, time);
  }

  /**
   * @return the time associated with the {@link FrameTransform} or {@code null}
   *         if there is no associated time
   */
  public Time getTime() {
    return time;
  }

  public geometry_msgs.TransformStamped toTransformStampedMessage(
      geometry_msgs.TransformStamped result) {
    Preconditions.checkNotNull(time);
    result.getHeader().setFrameId(target.toString());
    result.getHeader().setStamp(time);
    result.setChildFrameId(source.toString());
    transform.toTransformMessage(result.getTransform());
    return result;
  }

  @Override
  public String toString() {
    return String.format("FrameTransform<Source: %s, Target: %s, %s, Time: %s>", source, target,
        transform, time);
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((source == null) ? 0 : source.hashCode());
    result = prime * result + ((target == null) ? 0 : target.hashCode());
    result = prime * result + ((time == null) ? 0 : time.hashCode());
    result = prime * result + ((transform == null) ? 0 : transform.hashCode());
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
    FrameTransform other = (FrameTransform) obj;
    if (source == null) {
      if (other.source != null)
        return false;
    } else if (!source.equals(other.source))
      return false;
    if (target == null) {
      if (other.target != null)
        return false;
    } else if (!target.equals(other.target))
      return false;
    if (time == null) {
      if (other.time != null)
        return false;
    } else if (!time.equals(other.time))
      return false;
    if (transform == null) {
      if (other.transform != null)
        return false;
    } else if (!transform.equals(other.transform))
      return false;
    return true;
  }
}
