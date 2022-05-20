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

package org.ros.concurrent;

import com.google.common.collect.Maps;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.exception.RosRuntimeException;

import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletionService;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorCompletionService;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Wraps an {@link ScheduledExecutorService} to execute {@link Callable}s with
 * retries.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class RetryingExecutorService {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(RetryingExecutorService.class);

  private static final long DEFAULT_RETRY_DELAY = 5;
  private static final TimeUnit DEFAULT_RETRY_TIME_UNIT = TimeUnit.SECONDS;

  private final ScheduledExecutorService scheduledExecutorService;
  private final RetryLoop retryLoop;
  private final Map<Callable<Boolean>, CountDownLatch> latches;
  private final Map<Future<Boolean>, Callable<Boolean>> callables;
  private final CompletionService<Boolean> completionService;
  private final Object mutex;

  private long retryDelay;
  private TimeUnit retryTimeUnit;
  private boolean running;

  private class RetryLoop extends CancellableLoop {
    @Override
    public void loop() throws InterruptedException {
      Future<Boolean> future = completionService.take();
      Callable<Boolean> callable;
      CountDownLatch latch;
      // Grab the mutex to make sure submit() of the future that we took is finished.
      synchronized (mutex) {
        callable = callables.remove(future);
        latch = latches.get(callable);
      }
      boolean retry;
      try {
        retry = future.get();
      } catch (ExecutionException e) {
        throw new RosRuntimeException(e.getCause());
      }
      if (retry) {
        if (DEBUG) {
          log.info("Retry requested.");
        }
        final Callable<Boolean> finalCallable = callable;
        scheduledExecutorService.schedule(new Runnable() {
          @Override
          public void run() {
            submit(finalCallable);
          }
        }, retryDelay, retryTimeUnit);
      } else {
        latch.countDown();
      }
    }
  }

  /**
   * @param scheduledExecutorService
   *          the {@link ExecutorService} to wrap
   */
  public RetryingExecutorService(ScheduledExecutorService scheduledExecutorService) {
    this.scheduledExecutorService = scheduledExecutorService;
    retryLoop = new RetryLoop();
    latches = Maps.newConcurrentMap();
    callables = Maps.newConcurrentMap();
    completionService = new ExecutorCompletionService<Boolean>(scheduledExecutorService);
    mutex = new Object();
    retryDelay = DEFAULT_RETRY_DELAY;
    retryTimeUnit = DEFAULT_RETRY_TIME_UNIT;
    running = true;
    // TODO(damonkohler): Unify this with the passed in ExecutorService.
    scheduledExecutorService.execute(retryLoop);
  }

  /**
   * Submit a new {@link Callable} to be executed. The submitted
   * {@link Callable} should return {@code true} to be retried, {@code false}
   * otherwise.
   * 
   * @param callable
   *          the {@link Callable} to execute
   * @throws RejectedExecutionException
   *           if the {@link RetryingExecutorService} is shutting down
   */
  public void submit(Callable<Boolean> callable) {
    synchronized (mutex) {
      if (running) {
        Future<Boolean> future = completionService.submit(callable);
        latches.put(callable, new CountDownLatch(1));
        callables.put(future, callable);
      } else {
        throw new RejectedExecutionException();
      }
    }
  }

  /**
   * @param delay
   *          the delay in units of {@code unit}
   * @param unit
   *          the {@link TimeUnit} of the delay
   */
  public void setRetryDelay(long delay, TimeUnit unit) {
    retryDelay = delay;
    retryTimeUnit = unit;
  }

  /**
   * Stops accepting new {@link Callable}s and waits for all submitted
   * {@link Callable}s to finish within the specified timeout.
   * 
   * @param timeout
   *          the timeout in units of {@code unit}
   * @param unit
   *          the {@link TimeUnit} of {@code timeout}
   * @throws InterruptedException
   */
  public void shutdown(long timeout, TimeUnit unit) throws InterruptedException {
    running = false;
    for (CountDownLatch latch : latches.values()) {
      latch.await(timeout, unit);
    }
    retryLoop.cancel();
  }
}