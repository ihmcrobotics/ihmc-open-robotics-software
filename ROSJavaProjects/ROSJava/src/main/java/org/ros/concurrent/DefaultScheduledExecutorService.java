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

import com.google.common.collect.Lists;

import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/**
 * This wraps a {@link Executors#newCachedThreadPool()} and a
 * {@link Executors#newScheduledThreadPool(int)} to provide the functionality of
 * both in a single {@link ScheduledExecutorService}. This is necessary since
 * the {@link ScheduledExecutorService} uses an unbounded queue which makes it
 * impossible to create an unlimited number of threads on demand (as explained
 * in the {@link ThreadPoolExecutor} class javadoc.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class DefaultScheduledExecutorService implements ScheduledExecutorService {

  private static final int CORE_POOL_SIZE = 11;

  private final ExecutorService executorService;
  private final ScheduledExecutorService scheduledExecutorService;

  public DefaultScheduledExecutorService() {
    this(Executors.newCachedThreadPool());
  }

  /**
   * This instance will take over the lifecycle of the services.
   * 
   * @param executorService
   */
  public DefaultScheduledExecutorService(ExecutorService executorService) {
    this(executorService, Executors.newScheduledThreadPool(CORE_POOL_SIZE));
  }

  /**
   * This instance will take over the lifecycle of the services.
   * 
   * @param executorService
   * @param scheduledExecutorService
   */
  public DefaultScheduledExecutorService(ExecutorService executorService,
      ScheduledExecutorService scheduledExecutorService) {
    this.executorService = executorService;
    this.scheduledExecutorService = scheduledExecutorService;
  }

  @Override
  public void shutdown() {
    executorService.shutdown();
    scheduledExecutorService.shutdown();
  }

  @Override
  public List<Runnable> shutdownNow() {
    List<Runnable> combined = Lists.newArrayList();
    combined.addAll(executorService.shutdownNow());
    combined.addAll(scheduledExecutorService.shutdownNow());
    return combined;
  }

  @Override
  public boolean isShutdown() {
    return executorService.isShutdown() && scheduledExecutorService.isShutdown();
  }

  @Override
  public boolean isTerminated() {
    return executorService.isTerminated() && scheduledExecutorService.isTerminated();
  }

  /**
   * First calls {@link #awaitTermination(long, TimeUnit)} on the wrapped
   * {@link ExecutorService} and then {@link #awaitTermination(long, TimeUnit)}
   * on the wrapped {@link ScheduledExecutorService}.
   * 
   * @return {@code true} if both {@link Executor}s terminated, {@code false}
   *         otherwise
   */
  @Override
  public boolean awaitTermination(long timeout, TimeUnit unit) throws InterruptedException {
    boolean executorServiceResult = executorService.awaitTermination(timeout, unit);
    boolean scheduledExecutorServiceResult =
        scheduledExecutorService.awaitTermination(timeout, unit);
    return executorServiceResult && scheduledExecutorServiceResult;
  }

  @Override
  public <T> Future<T> submit(Callable<T> task) {
    return executorService.submit(task);
  }

  @Override
  public <T> Future<T> submit(Runnable task, T result) {
    return executorService.submit(task, result);
  }

  @Override
  public Future<?> submit(Runnable task) {
    return executorService.submit(task);
  }

  @Override
  public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks)
      throws InterruptedException {
    return executorService.invokeAll(tasks);
  }

  @Override
  public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks, long timeout,
      TimeUnit unit) throws InterruptedException {
    return executorService.invokeAll(tasks, timeout, unit);
  }

  @Override
  public <T> T invokeAny(Collection<? extends Callable<T>> tasks) throws InterruptedException,
      ExecutionException {
    return executorService.invokeAny(tasks);
  }

  @Override
  public <T> T invokeAny(Collection<? extends Callable<T>> tasks, long timeout, TimeUnit unit)
      throws InterruptedException, ExecutionException, TimeoutException {
    return executorService.invokeAny(tasks, timeout, unit);
  }

  @Override
  public void execute(Runnable command) {
    executorService.execute(command);
  }

  @Override
  public ScheduledFuture<?> schedule(Runnable command, long delay, TimeUnit unit) {
    return scheduledExecutorService.schedule(command, delay, unit);
  }

  @Override
  public <V> ScheduledFuture<V> schedule(Callable<V> callable, long delay, TimeUnit unit) {
    return scheduledExecutorService.schedule(callable, delay, unit);
  }

  @Override
  public ScheduledFuture<?> scheduleAtFixedRate(Runnable command, long initialDelay, long period,
      TimeUnit unit) {
    return scheduledExecutorService.scheduleAtFixedRate(command, initialDelay, period, unit);
  }

  @Override
  public ScheduledFuture<?> scheduleWithFixedDelay(Runnable command, long initialDelay, long delay,
      TimeUnit unit) {
    return scheduledExecutorService.scheduleWithFixedDelay(command, initialDelay, delay, unit);
  }
}
