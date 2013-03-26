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

import org.ros.node.topic.CountDownPublisherListener;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class CountDownRegistrantListener<T> implements RegistrantListener<T> {

  private final CountDownLatch masterRegistrationSuccessLatch;
  private final CountDownLatch masterRegistrationFailureLatch;
  private final CountDownLatch masterUnregistrationSuccessLatch;
  private final CountDownLatch masterUnregistrationFailureLatch;

  /**
   * Construct a {@link CountDownPublisherListener} with all counts set to 1.
   */
  public CountDownRegistrantListener() {
    this(1, 1, 1, 1);
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
  public CountDownRegistrantListener(int masterRegistrationSuccessCount,
      int masterRegistrationFailureCount, int masterUnregistrationSuccessCount,
      int masterUnregistrationFailureCount) {
    this(new CountDownLatch(masterRegistrationSuccessCount), new CountDownLatch(
        masterRegistrationFailureCount), new CountDownLatch(masterUnregistrationSuccessCount),
        new CountDownLatch(masterUnregistrationFailureCount));
  }

  public CountDownRegistrantListener(CountDownLatch masterRegistrationSuccessLatch,
      CountDownLatch masterRegistrationFailureLatch,
      CountDownLatch masterUnregistrationSuccessLatch,
      CountDownLatch masterUnregistrationFailureLatch) {
    this.masterRegistrationSuccessLatch = masterRegistrationSuccessLatch;
    this.masterRegistrationFailureLatch = masterRegistrationFailureLatch;
    this.masterUnregistrationSuccessLatch = masterUnregistrationSuccessLatch;
    this.masterUnregistrationFailureLatch = masterUnregistrationFailureLatch;
  }

  @Override
  public void onMasterRegistrationSuccess(T registrant) {
    masterRegistrationSuccessLatch.countDown();
  }

  @Override
  public void onMasterRegistrationFailure(T registrant) {
    masterRegistrationFailureLatch.countDown();
  }

  @Override
  public void onMasterUnregistrationSuccess(T registrant) {
    masterUnregistrationSuccessLatch.countDown();
  }

  @Override
  public void onMasterUnregistrationFailure(T registrant) {
    masterUnregistrationFailureLatch.countDown();
  }

  /**
   * Wait for the requested number of successful registrations.
   * 
   * @throws InterruptedException
   */
  public void awaitMasterRegistrationSuccess() throws InterruptedException {
    masterRegistrationSuccessLatch.await();
  }

  /**
   * Wait for the requested number of successful registrations within the given
   * time period.
   * 
   * @param timeout
   *          the maximum time to wait
   * @param unit
   *          the {@link TimeUnit} of the {@code timeout} argument
   * @return {@code true} if the registration happened within the time period,
   *         {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean awaitMasterRegistrationSuccess(long timeout, TimeUnit unit)
      throws InterruptedException {
    return masterRegistrationSuccessLatch.await(timeout, unit);
  }

  /**
   * Wait for the requested number of successful unregistrations.
   * 
   * @throws InterruptedException
   */
  public void awaitMasterUnregistrationSuccess() throws InterruptedException {
    masterUnregistrationSuccessLatch.await();
  }

  /**
   * Wait for the requested number of successful unregistrations within the
   * given time period.
   * 
   * @param timeout
   *          the maximum time to wait
   * @param unit
   *          the {@link TimeUnit} of the {@code timeout} argument
   * @return {@code true} if the unregistration happened within the time period,
   *         {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean awaitMasterUnregistrationSuccess(long timeout, TimeUnit unit)
      throws InterruptedException {
    return masterUnregistrationSuccessLatch.await(timeout, unit);
  }

  /**
   * Wait for the requested number of failed registrations.
   * 
   * @throws InterruptedException
   */
  public void awaitMasterRegistrationFailure() throws InterruptedException {
    masterRegistrationFailureLatch.await();
  }

  /**
   * Wait for the requested number of failed registrations within the given time
   * period.
   * 
   * @param timeout
   *          the maximum time to wait
   * @param unit
   *          the {@link TimeUnit} of the {@code timeout} argument
   * @return {@code true} if the registration happened within the time period,
   *         {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean awaitMasterRegistrationFailure(long timeout, TimeUnit unit)
      throws InterruptedException {
    return masterRegistrationFailureLatch.await(timeout, unit);
  }

  /**
   * Wait for the requested number of failed unregistrations.
   * 
   * @throws InterruptedException
   */
  public void awaitMasterUnregistrationFailure() throws InterruptedException {
    masterUnregistrationFailureLatch.await();
  }

  /**
   * Wait for the requested number of failed unregistrations within the given
   * time period.
   * 
   * @param timeout
   *          the maximum time to wait
   * @param unit
   *          the {@link TimeUnit} of the {@code timeout} argument
   * @return {@code true} if the unregistration happened within the time period,
   *         {@code false} otherwise
   * @throws InterruptedException
   */
  public boolean awaitMasterUnregistrationFailure(long timeout, TimeUnit unit)
      throws InterruptedException {
    return masterUnregistrationFailureLatch.await(timeout, unit);
  }
}
