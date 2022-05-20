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

import org.ros.internal.node.topic.TopicParticipant;
import org.ros.message.MessageListener;

import java.util.concurrent.TimeUnit;

/**
 * Subscribes to messages of a given type on a given ROS topic.
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <T>
 *          the {@link Subscriber} may only subscribe to messages of this type
 */
public interface Subscriber<T> extends TopicParticipant {

  /**
   * The message type given when a {@link Subscriber} chooses not to commit to a
   * specific message type.
   */
  public static final String TOPIC_MESSAGE_TYPE_WILDCARD = "*";

  /**
   * Adds a {@link MessageListener} to be called when new messages are received.
   * <p>
   * The {@link MessageListener} will be executed serially in its own thread. If
   * the {@link MessageListener} processes new messages slower than they arrive,
   * new messages will be queued up to the specified limit. Older messages are
   * removed from the buffer when the buffer limit is exceeded.
   * 
   * @param messageListener
   *          this {@link MessageListener} will be called when new messages are
   *          received
   * @param limit
   *          the maximum number of messages to buffer
   */
  void addMessageListener(MessageListener<T> messageListener, int limit);

  /**
   * Adds a {@link MessageListener} with a limit of 1.
   * 
   * @see #addMessageListener(MessageListener, int)
   */
  void addMessageListener(MessageListener<T> messageListener);

  /**
   * Removes a previously added {@link MessageListener}.
   * @param messageListener {@link MessageListener} to remove.
   * @return True if the listener was removed, false if it wasn't registered before.
   */
  boolean removeMessageListener(MessageListener<T> messageListener);

  /**
   * Removes all registered {@link MessageListener}s.
   */
  void removeAllMessageListeners();

  /**
   * Shuts down and unregisters the {@link Subscriber}. using the default
   * timeout Shutdown is delayed by at most the specified timeout to allow
   * {@link SubscriberListener#onShutdown(Subscriber)} callbacks to complete.
   * 
   * <p>
   * {@link SubscriberListener#onShutdown(Subscriber)} callbacks are executed in
   * separate threads.
   */
  void shutdown(long timeout, TimeUnit unit);

  /**
   * Shuts down and unregisters the {@link Subscriber} using the default timeout
   * for {@link SubscriberListener#onShutdown(Subscriber)} callbacks.
   * 
   * <p>
   * {@link SubscriberListener#onShutdown(Subscriber)} callbacks are executed in
   * separate threads.
   * 
   * @see Subscriber#shutdown(long, TimeUnit)
   */
  void shutdown();

  /**
   * Add a new lifecycle listener to the subscriber.
   * 
   * @param listener
   *          The listener to add.
   */
  void addSubscriberListener(SubscriberListener<T> listener);

  /**
   * @return {@code true} if the {@link Publisher} of this {@link Subscriber}'s
   *         topic is latched, {@code false} otherwise
   */
  boolean getLatchMode();
}
