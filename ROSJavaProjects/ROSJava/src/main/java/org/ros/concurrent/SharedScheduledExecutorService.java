package org.ros.concurrent;

import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/**
 * A {@link ScheduledExecutorService} which cannot be shut down. This can be
 * safely injected into instances which should not be in control of the
 * {@link ExecutorService}'s lifecycle.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class SharedScheduledExecutorService implements ScheduledExecutorService {

  /**
   * The scheduledExecutorService {@link ScheduledExecutorService}.
   */
  private ScheduledExecutorService scheduledExecutorService;

  public SharedScheduledExecutorService(ScheduledExecutorService wrapped) {
    this.scheduledExecutorService = wrapped;
  }

  /**
   * @see java.util.concurrent.ExecutorService#awaitTermination(long,
   *      java.util.concurrent.TimeUnit)
   */
  @Override
  public boolean awaitTermination(long timeout, TimeUnit unit) throws InterruptedException {
    return scheduledExecutorService.awaitTermination(timeout, unit);
  }

  /**
   * @param command
   * @see java.util.concurrent.Executor#execute(java.lang.Runnable)
   */
  @Override
  public void execute(Runnable command) {
    scheduledExecutorService.execute(command);
  }

  /**
   * @see java.util.concurrent.ExecutorService#invokeAll(java.util.Collection,
   *      long, java.util.concurrent.TimeUnit)
   */
  @Override
  public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks, long timeout,
      TimeUnit unit) throws InterruptedException {
    return scheduledExecutorService.invokeAll(tasks, timeout, unit);
  }

  /**
   * @see java.util.concurrent.ExecutorService#invokeAll(java.util.Collection)
   */
  @Override
  public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks)
      throws InterruptedException {
    return scheduledExecutorService.invokeAll(tasks);
  }

  /**
   * @see java.util.concurrent.ExecutorService#invokeAny(java.util.Collection,
   *      long, java.util.concurrent.TimeUnit)
   */
  @Override
  public <T> T invokeAny(Collection<? extends Callable<T>> tasks, long timeout, TimeUnit unit)
      throws InterruptedException, ExecutionException, TimeoutException {
    return scheduledExecutorService.invokeAny(tasks, timeout, unit);
  }

  /**
   * @see java.util.concurrent.ExecutorService#invokeAny(java.util.Collection)
   */
  @Override
  public <T> T invokeAny(Collection<? extends Callable<T>> tasks) throws InterruptedException,
      ExecutionException {
    return scheduledExecutorService.invokeAny(tasks);
  }

  /**
   * @see java.util.concurrent.ExecutorService#isShutdown()
   */
  @Override
  public boolean isShutdown() {
    return scheduledExecutorService.isShutdown();
  }

  /**
   * @see java.util.concurrent.ExecutorService#isTerminated()
   */
  @Override
  public boolean isTerminated() {
    return scheduledExecutorService.isTerminated();
  }

  /**
   * @see java.util.concurrent.ScheduledExecutorService#schedule(java.util.concurrent.Callable,
   *      long, java.util.concurrent.TimeUnit)
   */
  @Override
  public <V> ScheduledFuture<V> schedule(Callable<V> callable, long delay, TimeUnit unit) {
    return scheduledExecutorService.schedule(callable, delay, unit);
  }

  /**
   * @see java.util.concurrent.ScheduledExecutorService#schedule(java.lang.Runnable,
   *      long, java.util.concurrent.TimeUnit)
   */
  @Override
  public ScheduledFuture<?> schedule(Runnable command, long delay, TimeUnit unit) {
    return scheduledExecutorService.schedule(command, delay, unit);
  }

  /**
   * @see java.util.concurrent.ScheduledExecutorService#scheduleAtFixedRate(java.lang.Runnable,
   *      long, long, java.util.concurrent.TimeUnit)
   */
  @Override
  public ScheduledFuture<?> scheduleAtFixedRate(Runnable command, long initialDelay, long period,
      TimeUnit unit) {
    return scheduledExecutorService.scheduleAtFixedRate(command, initialDelay, period, unit);
  }

  /**
   * @see java.util.concurrent.ScheduledExecutorService#scheduleWithFixedDelay(java.lang.Runnable,
   *      long, long, java.util.concurrent.TimeUnit)
   */
  @Override
  public ScheduledFuture<?> scheduleWithFixedDelay(Runnable command, long initialDelay, long delay,
      TimeUnit unit) {
    return scheduledExecutorService.scheduleWithFixedDelay(command, initialDelay, delay, unit);
  }

  /**
   * @see java.util.concurrent.ExecutorService#shutdown()
   */
  @Override
  public void shutdown() {
    throw new UnsupportedOperationException("Cannot shut down service");
  }

  /**
   * @see java.util.concurrent.ExecutorService#shutdownNow()
   */
  @Override
  public List<Runnable> shutdownNow() {
    throw new UnsupportedOperationException("Cannot shut down service");
  }

  /**
   * @see java.util.concurrent.ExecutorService#submit(java.util.concurrent.Callable)
   */
  @Override
  public <T> Future<T> submit(Callable<T> task) {
    return scheduledExecutorService.submit(task);
  }

  /**
   * @see java.util.concurrent.ExecutorService#submit(java.lang.Runnable,
   *      java.lang.Object)
   */
  @Override
  public <T> Future<T> submit(Runnable task, T result) {
    return scheduledExecutorService.submit(task, result);
  }

  /**
   * @see java.util.concurrent.ExecutorService#submit(java.lang.Runnable)
   */
  @Override
  public Future<?> submit(Runnable task) {
    return scheduledExecutorService.submit(task);
  }
}