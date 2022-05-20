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

import org.ros.internal.node.topic.PublisherIdentifier;

import org.ros.internal.node.CountDownRegistrantListener;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * A {@link SubscriberListener} which uses separate {@link CountDownLatch}
 * instances for all messages.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class CountDownSubscriberListener<T> extends CountDownRegistrantListener<Subscriber<T>>
    implements SubscriberListener<T> {

  private final CountDownLatch shutdownLatch;
  private final CountDownLatch newPublisherLatch;

  /**
   * Construct a {@link CountDownSubscriberListener} with all counts set to 1.
   */
  public static <T> CountDownSubscriberListener<T> newDefault() {
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
   *          the number of counts to wait for for a new publisher
   */
  public static <T> CountDownSubscriberListener<T> newFromCounts(
      int masterRegistrationSuccessCount, int masterRegistrationFailureCount,
      int masterUnregistrationSuccessCount, int masterUnregistrationFailureCount,
      int newSubscriberCount) {
    return new CountDownSubscriberListener<T>(new CountDownLatch(masterRegistrationSuccessCount),
        new CountDownLatch(masterRegistrationFailureCount), new CountDownLatch(
            masterUnregistrationSuccessCount),
        new CountDownLatch(masterUnregistrationFailureCount),
        new CountDownLatch(newSubscriberCount));
  }

  private CountDownSubscriberListener(CountDownLatch masterRegistrationSuccessLatch,
      CountDownLatch masterRegistrationFailureLatch,
      CountDownLatch masterUnregistrationSuccessLatch,
      CountDownLatch masterUnregistrationFailureLatch, CountDownLatch newPublisherLatch) {
    super(masterRegistrationSuccessLatch, masterRegistrationFailureLatch,
        masterUnregistrationSuccessLatch, masterUnregistrationFailureLatch);
    this.newPublisherLatch = newPublisherLatch;
    shutdownLatch = new CountDownLatch(1);
  }

  @Override
  public void onNewPublisher(Subscriber<T> subscriber, PublisherIdentifier publisherIdentifier) {
    newPublisherLatch.countDown();
  }

  @Override
  public void onShutdown(Subscriber<T> subscriber) {
    shutdownLatch.countDown();
  }

  /**
   * Wait for the requested number of new publishers.
   * 
   * @throws InterruptedException
   */
  public void awaitNewPublisher() throws InterruptedException {
    newPublisherLatch.await();
  }

  /**
   * Wait for the requested number of new publishers within the given time
   * period.
   * 
   * @param timeout
   *          the maximum time to wait
   * @param unit
   *          the time unit of the {@code timeout} argument
   * @return {@code true} if the new publishers connected within the time
   *         period, {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean awaitNewPublisher(long timeout, TimeUnit unit) throws InterruptedException {
    return newPublisherLatch.await(timeout, unit);
  }

  /**
   * Wait for shutdown.
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
   *          the time unit of the {@code timeout} argument
   * @return {@code true} if the shutdowns happened within the time period,
   *         {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean awaitShutdown(long timeout, TimeUnit unit) throws InterruptedException {
    return shutdownLatch.await(timeout, unit);
  }
}
