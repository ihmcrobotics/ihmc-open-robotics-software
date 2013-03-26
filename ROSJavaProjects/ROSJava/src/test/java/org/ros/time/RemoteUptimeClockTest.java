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

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;
import org.ros.time.RemoteUptimeClock.LocalUptimeProvider;

import java.util.concurrent.Callable;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class RemoteUptimeClockTest {

  private double localUptime;
  private double remoteUptime;
  private double delta;
  private double drift;
  private double driftSensitivity;
  private double errorReductionCoefficientSensitivity;
  private int latencyOutlierFilterSampleSize;
  private double latencyOutlierFilterThreshold;

  @Before
  public void setup() {
    localUptime = 0;
    remoteUptime = 0;
  }

  private RemoteUptimeClock newRemoteUptimeClock() {
    RemoteUptimeClock remoteUptimeClock =
        new RemoteUptimeClock(new LocalUptimeProvider() {
          @Override
          public double getSeconds() {
            moveTimeForward(delta, drift);
            return localUptime;
          }
        }, new Callable<Double>() {
          @Override
          public Double call() throws Exception {
            return remoteUptime;
          }
        }, driftSensitivity, errorReductionCoefficientSensitivity, latencyOutlierFilterSampleSize,
            latencyOutlierFilterThreshold);
    return remoteUptimeClock;
  }

  private void moveTimeForward(double delta, double drift) {
    localUptime += delta;
    remoteUptime += delta / drift;
  }

  @Test
  public void testUnityDrift() throws Exception {
    delta = 7;
    drift = 1;
    driftSensitivity = 1;
    errorReductionCoefficientSensitivity = 1;
    latencyOutlierFilterSampleSize = 1;
    latencyOutlierFilterThreshold = 1.5;
    RemoteUptimeClock remoteUptimeClock = newRemoteUptimeClock();
    remoteUptimeClock.calibrate(10, 0);
    for (int i = 0; i < 10000; i++) {
      remoteUptimeClock.update();
      assertEquals(1, remoteUptimeClock.getDrift(), 1e-6);
    }
    assertEquals(0, remoteUptimeClock.getErrorReductionCoefficient(), 1e-9);
  }

  @Test
  public void testDrift() throws Exception {
    delta = 31;
    drift = 2;
    driftSensitivity = 1;
    errorReductionCoefficientSensitivity = 1;
    latencyOutlierFilterSampleSize = 1;
    latencyOutlierFilterThreshold = 1.5;
    RemoteUptimeClock remoteUptimeClock = newRemoteUptimeClock();
    remoteUptimeClock.calibrate(10, 0);
    for (int i = 0; i < 10000; i++) {
      remoteUptimeClock.update();
      assertEquals(2, remoteUptimeClock.getDrift(), 1e-6);
    }
    assertEquals(0, remoteUptimeClock.getErrorReductionCoefficient(), 1e-9);
  }

  @Test
  public void testConvergence() {
    delta = 7;
    drift = 2;
    driftSensitivity = 1;
    errorReductionCoefficientSensitivity = 1;
    latencyOutlierFilterSampleSize = 1;
    latencyOutlierFilterThreshold = 1.5;
    RemoteUptimeClock remoteUptimeClock = newRemoteUptimeClock();
    remoteUptimeClock.calibrate(10, 0);
    // Remote clock jumps.
    remoteUptime += 71;
    // We update less often.
    delta = 31;
    // Calibrated drift was wrong.
    drift = 6;
    for (int i = 0; i < 3; i++) {
      remoteUptimeClock.update();
    }
    assertEquals(6, remoteUptimeClock.getDrift(), 1e-6);
    for (int i = 0; i < 11; i++) {
      remoteUptimeClock.update();
      assertEquals(6, remoteUptimeClock.getDrift(), 1e-6);
    }
    assertEquals(0, remoteUptimeClock.getErrorReductionCoefficient(), 1e-9);
  }

  @Test
  public void testConvergenceSensitivity() {
    delta = 7;
    drift = 2;
    driftSensitivity = 0.5;
    errorReductionCoefficientSensitivity = 0.5;
    latencyOutlierFilterSampleSize = 1;
    latencyOutlierFilterThreshold = 1.5;
    RemoteUptimeClock remoteUptimeClock = newRemoteUptimeClock();
    remoteUptimeClock.calibrate(10, 0);
    // Calibrated drift was wrong.
    drift = 2.1;
    for (int i = 0; i < 11; i++) {
      remoteUptimeClock.update();
    }
    assertEquals(2.1, remoteUptimeClock.getDrift(), 1e-4);
    assertEquals(0, remoteUptimeClock.getErrorReductionCoefficient(), 1e-3);
  }
}
