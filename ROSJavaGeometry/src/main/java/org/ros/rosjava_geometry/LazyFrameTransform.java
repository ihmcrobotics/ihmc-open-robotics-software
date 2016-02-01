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

import com.google.common.annotations.VisibleForTesting;

/**
 * Lazily converts a {@link geometry_msgs.Transform} message to a
 * {@link Transform} on the first call to {@link #get()} and caches the result.
 * <p>
 * This class is thread-safe.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class LazyFrameTransform {

  private final geometry_msgs.TransformStamped message;

  // Avoiding constructor code duplication.
  private final Object mutex = new Object();

  private FrameTransform frameTransform;

  public LazyFrameTransform(geometry_msgs.TransformStamped message) {
    this.message = message;
  }

  @VisibleForTesting
  LazyFrameTransform(FrameTransform frameTransform) {
    message = null;
    this.frameTransform = frameTransform;
  }

  /**
   * @return the {@link FrameTransform} for the wrapped
   *         {@link geometry_msgs.TransformStamped} message
   */
  public FrameTransform get() {
    synchronized (mutex) {
      if (frameTransform != null) {
        return frameTransform;
      }
      frameTransform = FrameTransform.fromTransformStampedMessage(message);
    }
    return frameTransform;
  }
}