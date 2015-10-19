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

package org.ros.internal.node.topic;

import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * Listener for {@link TopicParticipantManager} events.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface TopicParticipantManagerListener {

  /**
   * Called when a new {@link Publisher} is added.
   * 
   * @param publisher
   *          the {@link Publisher} that was added
   */
  void onPublisherAdded(DefaultPublisher<?> publisher);

  /**
   * Called when a new {@link Publisher} is removed.
   * 
   * @param publisher
   *          the {@link Publisher} that was removed
   */
  void onPublisherRemoved(DefaultPublisher<?> publisher);

  /**
   * Called when a {@link Subscriber} is added.
   * 
   * @param subscriber
   *          the {@link Subscriber} that was added
   */
  void onSubscriberAdded(DefaultSubscriber<?> subscriber);

  /**
   * Called when a {@link Subscriber} is removed.
   * 
   * @param subscriber
   *          the {@link Subscriber} that was removed
   */
  void onSubscriberRemoved(DefaultSubscriber<?> subscriber);
}
