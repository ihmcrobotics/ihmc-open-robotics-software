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

import java.util.concurrent.TimeUnit;

/**
 * Publishes messages of a given type on a given ROS topic.
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <T>
 *          the {@link Publisher} may only publish messages of this type
 */
public interface Publisher<T> extends TopicParticipant {

  /**
   * @see <a
   *      href="http://www.ros.org/wiki/roscpp/Overview/Publishers%20and%20Subscribers#Publisher_Options">Publisher
   *      options documentation</a>
   * @param enabled
   *          {@code true} if published messages should be latched,
   *          {@code false} otherwise
   */
  void setLatchMode(boolean enabled);

  /**
   * @see <a
   *      href="http://www.ros.org/wiki/roscpp/Overview/Publishers%20and%20Subscribers#Publisher_Options">Publisher
   *      options documentation</a>
   * @return {@code true} if published messages will be latched, {@code false}
   *         otherwise
   */
  boolean getLatchMode();

  /**
   * Create a new message.
   * 
   * @return a new message
   */
  T newMessage();

  /**
   * Publishes a message. This message will be available on the topic that this
   * {@link Publisher} has been associated with.
   * 
   * @param message
   *          the message to publish
   */
  void publish(T message);

  /**
   * @return {@code true} if {@code getNumberOfSubscribers() > 0}, {@code false}
   *         otherwise
   */
  boolean hasSubscribers();

  /**
   * Get the number of {@link Subscriber}s currently connected to the
   * {@link Publisher}.
   * 
   * <p>
   * This counts the number of {@link Subscriber} registered. If a
   * {@link Subscriber} does not shutdown properly it will not be unregistered
   * and thus will contribute to this count.
   * 
   * @return the number of {@link Subscriber}s currently connected to the
   *         {@link Publisher}
   */
  int getNumberOfSubscribers();

  /**
   * Shuts down and unregisters the {@link Publisher}. Shutdown is delayed by at
   * most the specified timeout to allow
   * {@link PublisherListener#onShutdown(Publisher)} callbacks to complete.
   * 
   * <p>
   * {@link PublisherListener#onShutdown(Publisher)} callbacks are executed in
   * separate threads.
   */
  void shutdown(long timeout, TimeUnit unit);

  /**
   * Shuts down and unregisters the {@link Publisher} using the default timeout
   * for {@link PublisherListener#onShutdown(Publisher)} callbacks.
   * 
   * <p>
   * {@link PublisherListener#onShutdown(Publisher)} callbacks are executed in
   * separate threads.
   * 
   * @see Publisher#shutdown(long, TimeUnit)
   */
  void shutdown();

  /**
   * Add a new lifecycle listener to the {@link Publisher}.
   * 
   * @param listener
   *          the {@link PublisherListener} to add
   */
  void addListener(PublisherListener<T> listener);
}
