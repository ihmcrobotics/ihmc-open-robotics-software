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

import org.ros.internal.node.CountDownRegistrantListener;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * A {@link PublisherListener} which uses separate {@link CountDownLatch}
 * instances for all signals.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class CountDownPublisherListener<T> extends CountDownRegistrantListener<Publisher<T>>
    implements PublisherListener<T> {

  private final CountDownLatch shutdownLatch;
  private final CountDownLatch newSubscriberLatch;

  public static <T> CountDownPublisherListener<T> newDefault() {
    return newFromCounts(1, 1, 1, 1, 1);
  }

  /**
   * @param masterRegistrationSuccessCount
   *          the number of successful master registrations to wait for
   * @param masterRegistrationFailureCount
   *          the number of failing master registrations to wait for
   * @param masterUnregistrationSuccessCount
   *          the number of successful master unregistrations to wait for
   * @param masterUnregistrationFailureCount
   *          the number of failing master unregistrations to wait for
   * @param newSubscriberCount
   *          the number of new subscribers to wait for
   */
  public static <T> CountDownPublisherListener<T> newFromCounts(int masterRegistrationSuccessCount,
      int masterRegistrationFailureCount, int masterUnregistrationSuccessCount,
      int masterUnregistrationFailureCount, int newSubscriberCount) {
    return new CountDownPublisherListener<T>(new CountDownLatch(masterRegistrationSuccessCount),
        new CountDownLatch(masterRegistrationFailureCount), new CountDownLatch(
            masterUnregistrationSuccessCount),
        new CountDownLatch(masterUnregistrationFailureCount),
        new CountDownLatch(newSubscriberCount));
  }

  private CountDownPublisherListener(CountDownLatch masterRegistrationSuccessLatch,
      CountDownLatch masterRegistrationFailureLatch,
      CountDownLatch masterUnregistrationSuccessLatch,
      CountDownLatch masterUnregistrationFailureLatch, CountDownLatch newSubscriberLatch) {
    super(masterRegistrationSuccessLatch, masterRegistrationFailureLatch,
        masterUnregistrationSuccessLatch, masterUnregistrationFailureLatch);
    this.newSubscriberLatch = newSubscriberLatch;
    shutdownLatch = new CountDownLatch(1);
  }

  @Override
  public void onNewSubscriber(Publisher<T> publisher, SubscriberIdentifier subscriberIdentifier) {
    newSubscriberLatch.countDown();
  }

  @Override
  public void onShutdown(Publisher<T> publisher) {
    shutdownLatch.countDown();
  }

  /**
   * Wait for the requested number of shutdowns.
   * 
   * @throws InterruptedException
   */
  public void awaitNewSubscriber() throws InterruptedException {
    newSubscriberLatch.await();
  }

  /**
   * Wait for the requested number of new subscribers within the given time
   * period.
   * 
   * @param timeout
   *          the maximum time to wait
   * @param unit
   *          the {@link TimeUnit} of the {@code timeout} argument
   * @return {@code true} if the requested number of new subscribers connect
   *         within the time period {@code false} otherwise.
   * @throws InterruptedException
   */
  public boolean awaitNewSubscriber(long timeout, TimeUnit unit) throws InterruptedException {
    return newSubscriberLatch.await(timeout, unit);
  }

  /**
   * Wait for for shutdown.
   * 
   * @throws InterruptedException
   */
  public void awaitShutdown() throws InterruptedException {
    shutdownLatch.await();
  }

  /**
   * Wait for shutdown within the given time period.
   * 
   * @param timeout
   *          the maximum time to wait
   * @param unit
   *          the {@link TimeUnit} of the {@code timeout} argument
   * @return {@code true} if shutdown happened within the time period,
   *         {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean awaitShutdown(long timeout, TimeUnit unit) throws InterruptedException {
    return shutdownLatch.await(timeout, unit);
  }
}
