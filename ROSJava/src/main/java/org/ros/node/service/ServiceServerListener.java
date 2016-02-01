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

import org.ros.internal.node.RegistrantListener;

/**
 * A lifecycle listener for {@link ServiceServer} instances.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 * 
 * @param <T>
 *          the {@link ServiceServer} responds to requests of this type
 * @param <S>
 *          the {@link ServiceServer} returns responses of this type
 */
public interface ServiceServerListener<T, S> extends RegistrantListener<ServiceServer<T, S>> {

  /**
   * @param serviceServer
   *          the {@link ServiceServer} which has been shut down
   */
  void onShutdown(ServiceServer<T, S> serviceServer);
}
