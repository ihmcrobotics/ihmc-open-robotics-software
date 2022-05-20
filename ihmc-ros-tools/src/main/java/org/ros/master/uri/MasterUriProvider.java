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

package org.ros.master.uri;

import org.ros.exception.RosRuntimeException;

import java.net.URI;
import java.util.concurrent.TimeUnit;

/**
 * Provides URLs for a ROS master.
 * 
 * @author Keith M. Hughes
 */
public interface MasterUriProvider {

  /**
   * Get a master URI.
   * 
   * <p>
   * There is no guarantee that calling this class twice will provide the same
   * URI.
   * 
   * <p>
   * This call may or may not block until a URI is available.
   * 
   * @return a master URI
   * 
   * @throws RosRuntimeException
   *           this exception may or may not be thrown if there is no master URI
   *           available
   */
  URI getMasterUri() throws RosRuntimeException;

  /**
   * Get a master URI within a given amount of time.
   * 
   * <p>
   * There is no guarantee that calling this class twice will provide the same
   * URI.
   * 
   * <p>
   * This call may or may not block until a URI is available.
   * 
   * @param timeout
   *          the amount of time to wait for a URI
   * @param unit
   *          the time unit for the wait time
   * 
   * @return a master URI or {@code null} if none could be obtained within the
   *         timeout
   */
  URI getMasterUri(long timeout, TimeUnit unit);
}
