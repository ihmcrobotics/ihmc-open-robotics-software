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

import com.google.common.base.Preconditions;

import java.util.concurrent.ExecutorService;

/**
 * An interruptable loop that can be run by an {@link ExecutorService}.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public abstract class CancellableLoop implements Runnable {

  private final Object mutex;

  /**
   * {@code true} if the code has been run once, {@code false} otherwise.
   */
  private boolean ranOnce = false;

  /**
   * The {@link Thread} the code will be running in.
   */
  private Thread thread;

  public CancellableLoop() {
    mutex = new Object();
  }

  @Override
  public void run() {
    synchronized (mutex) {
      Preconditions.checkState(!ranOnce, "CancellableLoops cannot be restarted.");
      ranOnce = true;
      thread = Thread.currentThread();
    }
    try {
      setup();
      while (!thread.isInterrupted()) {
        loop();
      }
    } catch (InterruptedException e) {
      handleInterruptedException(e);
    } finally {
      thread = null;
    }
  }

  /**
   * The setup block for the loop. This will be called exactly once before
   * the first call to {@link #loop()}.
   */
  protected void setup() {
  }

  /**
   * The body of the loop. This will run continuously until the
   * {@link CancellableLoop} has been interrupted externally or by calling
   * {@link #cancel()}.
   */
  protected abstract void loop() throws InterruptedException;

  /**
   * An {@link InterruptedException} was thrown.
   */
  protected void handleInterruptedException(InterruptedException e) {
  }

  /**
   * Interrupts the loop.
   */
  public void cancel() {
    if (thread != null) {
      thread.interrupt();
    }
  }

  /**
   * @return {@code true} if the loop is running
   */
  public boolean isRunning() {
    return thread != null && !thread.isInterrupted();
  }
}
