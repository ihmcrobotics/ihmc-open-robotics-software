/*
 * Copyright (C) 2012 Google Inc.
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

package org.ros.concurrent;

import com.google.common.collect.Lists;

import java.util.Collection;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * A group of listeners.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ListenerGroup<T> {

  private final static int DEFAULT_QUEUE_CAPACITY = 128;

  private final ExecutorService executorService;
  private final Collection<EventDispatcher<T>> eventDispatchers;

  public ListenerGroup(ExecutorService executorService) {
    this.executorService = executorService;
    eventDispatchers = Lists.newCopyOnWriteArrayList();
  }

  /**
   * Adds a listener to the {@link ListenerGroup}.
   * 
   * @param listener
   *          the listener to add
   * @param queueCapacity
   *          the maximum number of events to buffer
   * @return the {@link EventDispatcher} responsible for calling the specified
   *         listener
   */
  public EventDispatcher<T> add(T listener, int queueCapacity) {
    EventDispatcher<T> eventDispatcher = new EventDispatcher<T>(listener, queueCapacity);
    eventDispatchers.add(eventDispatcher);
    executorService.execute(eventDispatcher);
    return eventDispatcher;
  }

  /**
   * Adds the specified listener to the {@link ListenerGroup} with the queue
   * limit set to {@link #DEFAULT_QUEUE_CAPACITY}.
   * 
   * @param listener
   *          the listener to add
   * @return the {@link EventDispatcher} responsible for calling the specified
   *         listener
   */
  public EventDispatcher<T> add(T listener) {
    return add(listener, DEFAULT_QUEUE_CAPACITY);
  }

  /**
   * Adds all the specified listeners to the {@link ListenerGroup}.
   * 
   * @param listeners
   *          the listeners to add
   * @param limit
   *          the maximum number of events to buffer
   * @return a {@link Collection} of {@link EventDispatcher}s responsible for
   *         calling the specified listeners
   */
  public Collection<EventDispatcher<T>> addAll(Collection<T> listeners, int limit) {
    Collection<EventDispatcher<T>> eventDispatchers = Lists.newArrayList();
    for (T listener : listeners) {
      eventDispatchers.add(add(listener, limit));
    }
    return eventDispatchers;
  }

  /**
   * Adds all the specified listeners to the {@link ListenerGroup} with the
   * queue capacity for each set to {@link Integer#MAX_VALUE}.
   * 
   * @param listeners
   *          the listeners to add
   * @return a {@link Collection} of {@link EventDispatcher}s responsible for
   *         calling the specified listeners
   */
  public Collection<EventDispatcher<T>> addAll(Collection<T> listeners) {
    return addAll(listeners, DEFAULT_QUEUE_CAPACITY);
  }

  /**
   * @return the number of listeners in the group
   */
  public int size() {
    return eventDispatchers.size();
  }

  /**
   * Signals all listeners.
   * <p>
   * Each {@link SignalRunnable} is executed in a separate thread.
   */
  public void signal(SignalRunnable<T> signalRunnable) {
    for (EventDispatcher<T> eventDispatcher : eventDispatchers) {
      eventDispatcher.signal(signalRunnable);
    }
  }

  /**
   * Signals all listeners and waits for the result.
   * <p>
   * Each {@link SignalRunnable} is executed in a separate thread. In the event
   * that the {@link SignalRunnable} is be dropped from the
   * {@link EventDispatcher}'s queue and thus not executed, this method will
   * block for the entire specified timeout.
   * 
   * @return {@code true} if all listeners completed within the specified time
   *         limit, {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean signal(final SignalRunnable<T> signalRunnable, long timeout, TimeUnit unit)
      throws InterruptedException {
    Collection<EventDispatcher<T>> copy = Lists.newArrayList(eventDispatchers);
    final CountDownLatch latch = new CountDownLatch(copy.size());
    for (EventDispatcher<T> eventDispatcher : copy) {
      eventDispatcher.signal(new SignalRunnable<T>() {
        @Override
        public void run(T listener) {
          signalRunnable.run(listener);
          latch.countDown();
        }
      });
    }
    return latch.await(timeout, unit);
  }

  public void shutdown() {
    for (EventDispatcher<T> eventDispatcher : eventDispatchers) {
      eventDispatcher.cancel();
    }
  }
}
