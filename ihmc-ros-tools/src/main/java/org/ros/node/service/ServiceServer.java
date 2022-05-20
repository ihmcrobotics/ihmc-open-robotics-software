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

package org.ros.node.service;

import org.ros.namespace.GraphName;

import java.net.URI;

/**
 * Provides a ROS service.
 * 
 * @see <a href="http://www.ros.org/wiki/Services">Services documentation</a>
 * 
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <T>
 *          this {@link ServiceServer} responds to requests of this type
 * @param <S>
 *          this {@link ServiceServer} returns responses of this type
 */
public interface ServiceServer<T, S> {

  /**
   * @return the name of the {@link ServiceServer}
   */
  GraphName getName();

  /**
   * @return the {@link URI} for this {@link ServiceServer}
   */
  URI getUri();

  /**
   * Stops the service and unregisters it.
   */
  void shutdown();

  /**
   * Add a {@link ServiceServerListener}.
   * 
   * @param listener
   *          the {@link ServiceServerListener} to add
   */
  void addListener(ServiceServerListener<T, S> listener);
}
