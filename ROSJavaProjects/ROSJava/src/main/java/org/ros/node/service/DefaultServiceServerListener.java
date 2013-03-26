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

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class DefaultServiceServerListener<T, S> implements ServiceServerListener<T, S> {

  @Override
  public void onMasterRegistrationSuccess(ServiceServer<T, S> registrant) {
  }

  @Override
  public void onMasterRegistrationFailure(ServiceServer<T, S> registrant) {
  }

  @Override
  public void onMasterUnregistrationSuccess(ServiceServer<T, S> registrant) {
  }

  @Override
  public void onMasterUnregistrationFailure(ServiceServer<T, S> registrant) {
  }

  @Override
  public void onShutdown(ServiceServer<T, S> serviceServer) {
  }
}
