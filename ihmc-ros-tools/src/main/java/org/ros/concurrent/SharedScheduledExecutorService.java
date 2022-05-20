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
   * @see ExecutorService#awaitTermination(long,
   *      TimeUnit)
   */
  @Override
  public boolean awaitTermination(long timeout, TimeUnit unit) throws InterruptedException {
    return scheduledExecutorService.awaitTermination(timeout, unit);
  }

  /**
   * @param command
   * @see java.util.concurrent.Executor#execute(Runnable)
   */
  @Override
  public void execute(Runnable command) {
    scheduledExecutorService.execute(command);
  }

  /**
   * @see ExecutorService#invokeAll(Collection,
   *      long, TimeUnit)
   */
  @Override
  public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks, long timeout,
      TimeUnit unit) throws InterruptedException {
    return scheduledExecutorService.invokeAll(tasks, timeout, unit);
  }

  /**
   * @see ExecutorService#invokeAll(Collection)
   */
  @Override
  public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks)
      throws InterruptedException {
    return scheduledExecutorService.invokeAll(tasks);
  }

  /**
   * @see ExecutorService#invokeAny(Collection,
   *      long, TimeUnit)
   */
  @Override
  public <T> T invokeAny(Collection<? extends Callable<T>> tasks, long timeout, TimeUnit unit)
      throws InterruptedException, ExecutionException, TimeoutException {
    return scheduledExecutorService.invokeAny(tasks, timeout, unit);
  }

  /**
   * @see ExecutorService#invokeAny(Collection)
   */
  @Override
  public <T> T invokeAny(Collection<? extends Callable<T>> tasks) throws InterruptedException,
      ExecutionException {
    return scheduledExecutorService.invokeAny(tasks);
  }

  /**
   * @see ExecutorService#isShutdown()
   */
  @Override
  public boolean isShutdown() {
    return scheduledExecutorService.isShutdown();
  }

  /**
   * @see ExecutorService#isTerminated()
   */
  @Override
  public boolean isTerminated() {
    return scheduledExecutorService.isTerminated();
  }

  /**
   * @see ScheduledExecutorService#schedule(Callable,
   *      long, TimeUnit)
   */
  @Override
  public <V> ScheduledFuture<V> schedule(Callable<V> callable, long delay, TimeUnit unit) {
    return scheduledExecutorService.schedule(callable, delay, unit);
  }

  /**
   * @see ScheduledExecutorService#schedule(Runnable,
   *      long, TimeUnit)
   */
  @Override
  public ScheduledFuture<?> schedule(Runnable command, long delay, TimeUnit unit) {
    return scheduledExecutorService.schedule(command, delay, unit);
  }

  /**
   * @see ScheduledExecutorService#scheduleAtFixedRate(Runnable,
   *      long, long, TimeUnit)
   */
  @Override
  public ScheduledFuture<?> scheduleAtFixedRate(Runnable command, long initialDelay, long period,
      TimeUnit unit) {
    return scheduledExecutorService.scheduleAtFixedRate(command, initialDelay, period, unit);
  }

  /**
   * @see ScheduledExecutorService#scheduleWithFixedDelay(Runnable,
   *      long, long, TimeUnit)
   */
  @Override
  public ScheduledFuture<?> scheduleWithFixedDelay(Runnable command, long initialDelay, long delay,
      TimeUnit unit) {
    return scheduledExecutorService.scheduleWithFixedDelay(command, initialDelay, delay, unit);
  }

  /**
   * @see ExecutorService#shutdown()
   */
  @Override
  public void shutdown() {
    throw new UnsupportedOperationException("Cannot shut down service");
  }

  /**
   * @see ExecutorService#shutdownNow()
   */
  @Override
  public List<Runnable> shutdownNow() {
    throw new UnsupportedOperationException("Cannot shut down service");
  }

  /**
   * @see ExecutorService#submit(Callable)
   */
  @Override
  public <T> Future<T> submit(Callable<T> task) {
    return scheduledExecutorService.submit(task);
  }

  /**
   * @see ExecutorService#submit(Runnable,
   *      Object)
   */
  @Override
  public <T> Future<T> submit(Runnable task, T result) {
    return scheduledExecutorService.submit(task, result);
  }

  /**
   * @see ExecutorService#submit(Runnable)
   */
  @Override
  public Future<?> submit(Runnable task) {
    return scheduledExecutorService.submit(task);
  }
}