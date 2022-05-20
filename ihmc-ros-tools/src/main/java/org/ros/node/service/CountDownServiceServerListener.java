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

import org.ros.internal.node.CountDownRegistrantListener;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * A {@link ServiceServerListener} which uses {@link CountDownLatch} to track
 * message invocations.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class CountDownServiceServerListener<T, S> extends
    CountDownRegistrantListener<ServiceServer<T, S>> implements ServiceServerListener<T, S> {

  private final CountDownLatch shutdownLatch;

  /**
   * Construct a {@link CountDownServiceServerListener} with all counts set to
   * 1.
   */
  public static <T, S> CountDownServiceServerListener<T, S> newDefault() {
    return newFromCounts(1, 1, 1, 1);
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
   */
  public static <T, S> CountDownServiceServerListener<T, S> newFromCounts(
      int masterRegistrationSuccessCount, int masterRegistrationFailureCount,
      int masterUnregistrationSuccessCount, int masterUnregistrationFailureCount) {
    return new CountDownServiceServerListener<T, S>(new CountDownLatch(
        masterRegistrationSuccessCount), new CountDownLatch(masterRegistrationFailureCount),
        new CountDownLatch(masterUnregistrationSuccessCount), new CountDownLatch(
            masterUnregistrationFailureCount));
  }

  private CountDownServiceServerListener(CountDownLatch masterRegistrationSuccessLatch,
      CountDownLatch masterRegistrationFailureLatch,
      CountDownLatch masterUnregistrationSuccessLatch,
      CountDownLatch masterUnregistrationFailureLatch) {
    super(masterRegistrationSuccessLatch, masterRegistrationFailureLatch,
        masterUnregistrationSuccessLatch, masterUnregistrationFailureLatch);
    shutdownLatch = new CountDownLatch(1);
  }

  @Override
  public void onShutdown(ServiceServer<T, S> server) {
    shutdownLatch.countDown();
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
   * @return {@code true} if shutdown happened within the time period,
   *         {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean awaitShutdown(long timeout, TimeUnit unit) throws InterruptedException {
    return shutdownLatch.await(timeout, unit);
  }
}
