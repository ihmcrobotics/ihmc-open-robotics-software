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

package org.ros.internal.node;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface RegistrantListener<T> {

  /**
   * The registrant has been registered with the master.
   * 
   * @param registrant
   *          the registrant which has been registered
   */
  void onMasterRegistrationSuccess(T registrant);

  /**
   * The registrant has failed to register with the master.
   * 
   * <p>
   * This may be called multiple times per registrant since master registration
   * will be retried until success.
   * 
   * @param registrant
   *          the registrant which has been registered
   */
  void onMasterRegistrationFailure(T registrant);

  /**
   * The registrant has been unregistered with the master.
   * 
   * @param registrant
   *          the registrant which has been unregistered
   */
  void onMasterUnregistrationSuccess(T registrant);

  /**
   * The registrant has failed to unregister with the master.
   * 
   * <p>
   * This may be called multiple times per registrant since master
   * unregistration will be retried until success.
   * 
   * @param registrant
   *          the registrant which has been unregistered
   */
  void onMasterUnregistrationFailure(T registrant);
}