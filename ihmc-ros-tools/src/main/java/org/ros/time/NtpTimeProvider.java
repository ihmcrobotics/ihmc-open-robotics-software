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

package org.ros.time;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.apache.commons.net.ntp.NTPUDPClient;
import org.apache.commons.net.ntp.TimeInfo;
import org.ros.math.CollectionMath;
import org.ros.message.Duration;
import org.ros.message.Time;

import java.io.IOException;
import java.net.InetAddress;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * Provides NTP synchronized wallclock (actual) time.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class NtpTimeProvider implements TimeProvider {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(NtpTimeProvider.class);

  private int sampleSize = 11;
  
  private final InetAddress host;
  private final ScheduledExecutorService scheduledExecutorService;
  private final WallTimeProvider wallTimeProvider;
  private final NTPUDPClient ntpClient;

  private long offset;
  private ScheduledFuture<?> scheduledFuture;

  /**
   * @param host
   *          the NTP host to use
   */
  public NtpTimeProvider(InetAddress host, ScheduledExecutorService scheduledExecutorService) {
    this.host = host;
    this.scheduledExecutorService = scheduledExecutorService;
    wallTimeProvider = new WallTimeProvider();
    ntpClient = new NTPUDPClient();
    ntpClient.setDefaultTimeout(500); // timeout to 500ms
    offset = 0;
    scheduledFuture = null;
  }

  /**
   * Update the current time offset from the configured NTP host.
   * 
   * @throws IOException : if ntpClient.getTime() fails too often.
   */
  public void updateTime() throws IOException {
    List<Long> offsets = Lists.newArrayList();
    int failures = 0;
    for (int i = 0; i < sampleSize; i++) {
      try { 
    	  offsets.add(computeOffset());
      } catch (IOException e) {
    	  if (++failures > sampleSize / 2) {
            throw e;
    	  }
      }
    }
    offset = CollectionMath.median(offsets);
    log.info(String.format("NTP time offset: %d ms", offset));
  }

  private long computeOffset() throws IOException {
    if (DEBUG) {
      log.info("Updating time offset from NTP server: " + host.getHostName());
    }
    TimeInfo time;
    try {
      time = ntpClient.getTime(host);
    } catch (IOException e) {
      if (DEBUG) {
        log.error("Failed to read time from NTP server: " + host.getHostName(), e);
      }
      throw e;
    }
    time.computeDetails();
    return time.getOffset();
  }

  /**
   * Starts periodically updating the current time offset periodically.
   * 
   * <p>
   * The first time update happens immediately.
   * 
   * <p>
   * Note that errors thrown while periodically updating time will be logged but
   * not rethrown.
   * 
   * @param period
   *          time between updates
   * @param unit
   *          unit of period
   */
  public void startPeriodicUpdates(long period, TimeUnit unit) {
    scheduledFuture =
        scheduledExecutorService.scheduleAtFixedRate(new Runnable() {
          @Override
          public void run() {
            try {
              updateTime();
            } catch (IOException e) {
              log.error("Periodic NTP update failed.", e);
            }
          }
        }, 0, period, unit);
  }

  /**
   * Stops periodically updating the current time offset.
   */
  public void stopPeriodicUpdates() {
    Preconditions.checkNotNull(scheduledFuture);
    scheduledFuture.cancel(true);
    scheduledFuture = null;
  }

  @Override
  public Time getCurrentTime() {
    Time currentTime = wallTimeProvider.getCurrentTime();
    return currentTime.add(Duration.fromMillis(offset));
  }

  /**
   * Sets how many samples will be taken from the NTP server
   * when calling {@link #updateTime()}.
   * @param sampleSize Number of samples to take. It has to be > 0.
   */
  public void setUpdateTimeSampleSize(int sampleSize) {
    Preconditions.checkArgument(sampleSize > 0);
    this.sampleSize = sampleSize;
  }
}
