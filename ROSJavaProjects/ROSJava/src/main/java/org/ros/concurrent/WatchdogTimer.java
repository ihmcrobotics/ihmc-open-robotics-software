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

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * A {@link WatchdogTimer} expects to receive a {@link #pulse()} at least once
 * every {@link #period} {@link #unit}s. Once per every period in which a
 * {@link #pulse()} is not received, the provided {@link Runnable} will be
 * executed.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class WatchdogTimer {

  private final ScheduledExecutorService scheduledExecutorService;
  private final long period;
  private final TimeUnit unit;
  private final Runnable runnable;

  private boolean pulsed;
  private ScheduledFuture<?> scheduledFuture;

  public WatchdogTimer(ScheduledExecutorService scheduledExecutorService, long period,
      TimeUnit unit, final Runnable runnable) {
    this.scheduledExecutorService = scheduledExecutorService;
    this.period = period;
    this.unit = unit;
    this.runnable = new Runnable() {
      @Override
      public void run() {
        try {
          if (!pulsed) {
            runnable.run();
          }
        } finally {
          pulsed = false;
        }
      }
    };
    pulsed = false;
  }

  public void start() {
    scheduledFuture = scheduledExecutorService.scheduleAtFixedRate(runnable, period, period, unit);
  }

  public void pulse() {
    pulsed = true;
  }

  public void cancel() {
    scheduledFuture.cancel(true);
  }
}
