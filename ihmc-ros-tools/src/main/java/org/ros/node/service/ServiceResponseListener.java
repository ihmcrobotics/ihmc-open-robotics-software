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

import org.ros.exception.RemoteException;

/**
 * A listener for service responses.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <MessageType>
 *          handles messages of this type
 */
public interface ServiceResponseListener<MessageType> {

  /**
   * Called when a service method returns successfully.
   * 
   * @param response
   *          the response message
   */
  void onSuccess(MessageType response);

  /**
   * Called when a service method fails to return successfully.
   * 
   * @param e
   *          the {@link RemoteException} received from the service
   */
  void onFailure(RemoteException e);

}
