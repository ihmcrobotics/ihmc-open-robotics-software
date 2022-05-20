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

package org.ros.node.topic;

import org.ros.internal.node.topic.SubscriberIdentifier;

/**
 * A {@link PublisherListener} which provides empty defaults for all signals.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class DefaultPublisherListener<T> implements PublisherListener<T> {

  @Override
  public void onMasterRegistrationSuccess(Publisher<T> publisher) {
  }

  @Override
  public void onMasterRegistrationFailure(Publisher<T> publisher) {
  }

  @Override
  public void onMasterUnregistrationSuccess(Publisher<T> publisher) {
  }

  @Override
  public void onMasterUnregistrationFailure(Publisher<T> publisher) {
  }

  @Override
  public void onNewSubscriber(Publisher<T> publisher, SubscriberIdentifier subscriberIdentifier) {
  }

  @Override
  public void onShutdown(Publisher<T> publisher) {
  }
}
