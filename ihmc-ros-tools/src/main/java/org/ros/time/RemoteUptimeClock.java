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

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.exception.RosRuntimeException;

import java.util.Collections;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.Callable;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class RemoteUptimeClock {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(RemoteUptimeClock.class);

  private final LocalUptimeProvider localUptimeProvider;
  private final Callable<Double> callable;
  private final LatencyOutlierFilter latencyOutlierFilter;

  /**
   * Sensitivity values are used to sampleSize the effect of jitter. The value
   * should be in the range [0, 1] where 0 indicates that the current estimate
   * will never change (i.e. new measurements have no effect on estimates) and 1
   * indicates that previous estimates have no effect on changes to the current
   * estimate.
   */
  private final double driftSensitivity;

  /**
   * @see #driftSensitivity
   */
  private final double errorReductionCoefficientSensitivity;

  private double localUptime;

  /**
   * Remote uptime is tracked as a pair of values: our previous measurement and
   * our prediction based on estimated drift.
   */
  private double measuredRemoteUptime;

  /**
   * @see #measuredRemoteUptime
   */
  private double predictedRemoteUptime;

  /**
   * Drift is measured in local uptime ticks per remote uptime tick.
   * 
   * @see #calculateDrift(double, double)
   */
  private double drift;

  /**
   * With {@link #drift} alone, it is possible to accumulate a constant error
   * that will never be corrected for. The {@link #errorReductionCoefficient} is
   * an additional term for removing this error.
   */
  private double errorReductionCoefficient;

  /**
   * Represents a tuple of measurement values that represent a single point in
   * time.
   */
  private final class UptimeCalculationResult {

    final double newLocalUptime;
    final double newRemoteUptime;
    final double latency;

    public UptimeCalculationResult(double newLocalUptime, double newRemoteUptime, double latency) {
      this.newLocalUptime = newLocalUptime;
      this.newRemoteUptime = newRemoteUptime;
      this.latency = latency;
    }
  }

  /**
   * Uses a sliding window and percentile range to detect latency outliers.
   * 
   * <p>
   * When receiving remote uptime measurements, the latency of the measurement
   * is used to estimate the local uptime at the point when the remote uptime
   * was measured. This calculation assumes that any measurement latency is
   * symmetrical. The larger the latency, the larger the potential error in our
   * estimate of local uptime at the measured remote uptime.
   * 
   * <p>
   * To reduce the effect of measurements with higher uncertainty, we filter out
   * measurements with latencies that exceed the specified percentile within our
   * sliding window.
   */
  private final class LatencyOutlierFilter {

    private final int sampleSize;
    private final double threshold;
    private final Queue<Double> latencies;

    public LatencyOutlierFilter(int sampleSize, double threshold) {
      Preconditions.checkArgument(sampleSize > 0);
      Preconditions.checkArgument(threshold > 1);
      this.threshold = threshold;
      this.sampleSize = sampleSize;
      latencies = Lists.newLinkedList();
    }

    /**
     * @param latency
     * @return {@code true} if the provided latency is outside the configured
     *         percentile, {@code false} otherwise
     */
    public boolean add(double latency) {
      latencies.add(latency);
      if (latencies.size() > sampleSize) {
        latencies.remove();
      } else {
        // Until the sliding window is full, we cannot reliably detect outliers.
        return false;
      }
      double medianLatency = getMedian();
      if (latency < medianLatency * threshold) {
        return false;
      }
      return true;
    }

    public double getMedian() {
      List<Double> ordered = Lists.newArrayList(latencies);
      Collections.sort(ordered);
      return ordered.get(latencies.size() / 2);
    }
  }

  @VisibleForTesting
  interface LocalUptimeProvider {
    double getSeconds();
  }

  /**
   * The provided {@link Callable} should return the current
   * measuredRemoteUptime of the remote clock with minimal overhead since the
   * run time of this call will be used to further improve the estimation of
   * measuredRemoteUptime.
   * 
   * @param timeProvider
   *          the local time provider
   * @param callable
   *          returns the current remote uptime in arbitrary units
   * @param driftSensitivity
   *          the sensitivity to drift adjustments, must be in the range [0, 1]
   * @param errorReductionCoefficientSensitivity
   *          the sensitivity to error reduction coefficient adjustments, must
   *          be in the range [0, 1]
   * @return a new {@link RemoteUptimeClock}
   */
  public static RemoteUptimeClock newDefault(final TimeProvider timeProvider,
      Callable<Double> callable, double driftSensitivity,
      double errorReductionCoefficientSensitivity, int latencyOutlierFilterSampleSize,
      double latencyOutlierFilterThreshold) {
    return new RemoteUptimeClock(new LocalUptimeProvider() {
      @Override
      public double getSeconds() {
        return timeProvider.getCurrentTime().toSeconds();
      }
    }, callable, driftSensitivity, errorReductionCoefficientSensitivity,
        latencyOutlierFilterSampleSize, latencyOutlierFilterThreshold);
  }

  @VisibleForTesting
  RemoteUptimeClock(LocalUptimeProvider localUptimeProvider, Callable<Double> callable,
      double driftSensitivity, double errorReductionCoefficientSensitivity,
      int latencyOutlierFilterSampleSize, double latencyOutlierFilterThreshold) {
    Preconditions.checkArgument(driftSensitivity >= 0 && driftSensitivity <= 1);
    Preconditions.checkArgument(errorReductionCoefficientSensitivity >= 0
        && errorReductionCoefficientSensitivity <= 1);
    this.localUptimeProvider = localUptimeProvider;
    this.callable = callable;
    this.driftSensitivity = driftSensitivity;
    this.errorReductionCoefficientSensitivity = errorReductionCoefficientSensitivity;
    latencyOutlierFilter =
        new LatencyOutlierFilter(latencyOutlierFilterSampleSize, latencyOutlierFilterThreshold);
    errorReductionCoefficient = 0;
  }

  /**
   * Good calibration settings will depend on the remote uptime provider. In
   * general, choosing a sample size around 10 and a delay that is large enough
   * to include more than 100 uptime ticks will give reasonable results.
   * 
   * @param sampleSize
   *          the number of samples to use for calibration
   * @param samplingDelayMillis
   *          the delay in milliseconds between collecting each sample
   */
  public void calibrate(int sampleSize, double samplingDelayMillis) {
    log.info("Starting calibration...");
    double remoteUptimeSum = 0;
    double localUptimeSum = 0;
    double driftSum = 0;
    for (int i = 0; i < sampleSize; i++) {
      UptimeCalculationResult result = calculateNewUptime(callable);
      latencyOutlierFilter.add(result.latency);
      if (i > 0) {
        double localUptimeDelta = result.newLocalUptime - localUptime;
        double remoteUptimeDelta = result.newRemoteUptime - measuredRemoteUptime;
        driftSum += calculateDrift(localUptimeDelta, remoteUptimeDelta);
      }
      measuredRemoteUptime = result.newRemoteUptime;
      localUptime = result.newLocalUptime;
      remoteUptimeSum += measuredRemoteUptime;
      localUptimeSum += localUptime;
      try {
        Thread.sleep((long) samplingDelayMillis);
      } catch (InterruptedException e) {
        throw new RosRuntimeException(e);
      }
    }
    // We have n samples, but n - 1 intervals. errorReductionCoefficient is the
    // average interval magnitude.
    drift = driftSum / (sampleSize - 1);
    // If localUptime == -offset then measuredRemoteUptime == 0 (e.g. if
    // localUptime is 10s and measuredRemoteUptime is 5s, then offset should be
    // -5s since the localUptime started 5s earlier than measuredRemoteUptime).
    double offset = (drift * remoteUptimeSum - localUptimeSum) / sampleSize;
    predictedRemoteUptime = (localUptime + offset) / drift;
    log.info(String.format("Calibration complete. Drift: %.4g, Offset: %.4f s", drift, offset));
  }

  /**
   * @see #drift
   * 
   * @param localUptimeDelta
   *          the delta between the two local uptimes that correspond to the two
   *          remote uptimes used to determine {@code remoteUptimeDelta}
   * @param remoteUptimeDelta
   *          the delta between the two remote uptimes that correspond to the
   *          two local uptimes used to determine {@code localUptimeDelta}
   * @return the calculated drift
   */
  private double calculateDrift(double localUptimeDelta, double remoteUptimeDelta) {
    Preconditions.checkState(remoteUptimeDelta > 1e-9);
    return localUptimeDelta / remoteUptimeDelta;
  }

  /**
   * Update this {@link RemoteUptimeClock} with the latest uptime from the
   * remote clock.
   * 
   * <p>
   * This will update internal estimates of drift and error. Ideally, it should
   * be called periodically with a consistent time interval between updates
   * (e.g. 10 seconds).
   */
  public void update() {
    UptimeCalculationResult result = calculateNewUptime(callable);
    double newLocalUptime = result.newLocalUptime;
    double newRemoteUptime = result.newRemoteUptime;
    double latency = result.latency;

    if (latencyOutlierFilter.add(latency)) {
      log.warn(String.format(
          "Measurement latency marked as outlier. Latency: %.4f s, Median: %.4f s", latency,
          latencyOutlierFilter.getMedian()));
      return;
    }

    double localUptimeDelta = newLocalUptime - localUptime;
    double remoteUptimeDelta = newRemoteUptime - measuredRemoteUptime;
    Preconditions.checkState(localUptimeDelta > 1e-9);
    Preconditions.checkState(remoteUptimeDelta > 1e-9);
    if (DEBUG) {
      log.info(String.format("localUptimeDelta: %.4g, remoteUptimeDelta: %.4g", localUptimeDelta,
          remoteUptimeDelta));
    }

    double newDrift =
        driftSensitivity * (localUptimeDelta / remoteUptimeDelta) + (1 - driftSensitivity) * drift;
    // Non-jumping behavior from (localUptime, predictedRemoteUptime) to
    // (newLocalUptime, newAdjustedRemoteUptime). Note that it does not depend
    // directly on measuredRemoteUptime or newRemoteUptime.
    double newPredictedRemoteUptime =
        predictedRemoteUptime + (localUptimeDelta / (drift + errorReductionCoefficient));
    double nextPredictedRemoteUptime = newRemoteUptime + remoteUptimeDelta;
    double newCombinedDriftAndError =
        localUptimeDelta / (nextPredictedRemoteUptime - newPredictedRemoteUptime);
    double newErrorReductionCoefficient =
        errorReductionCoefficientSensitivity * (newCombinedDriftAndError - newDrift);
    double deltaRatio = remoteUptimeDelta / localUptimeDelta;
    double error = newLocalUptime - toLocalUptime(newRemoteUptime);
    log.info(String.format("Latency: %.4f s, Delta ratio: %.4f, Drift: %.4g, "
        + "Error reduction coefficient: %.4g, Error: %.4f s", latency, deltaRatio, newDrift,
        newErrorReductionCoefficient, error));

    measuredRemoteUptime = newRemoteUptime;
    predictedRemoteUptime = newPredictedRemoteUptime;
    localUptime = newLocalUptime;
    drift = newDrift;
    errorReductionCoefficient = newErrorReductionCoefficient;
  }

  /**
   * Creates a new {@link UptimeCalculationResult} where the local uptime has
   * been adjusted to compensate for latency while retrieving the remote uptime.
   * 
   * @param callable
   *          returns the remote uptime as quickly as possible
   * @return a new {@link UptimeCalculationResult}
   */
  private UptimeCalculationResult calculateNewUptime(Callable<Double> callable) {
    double newLocalUptime = localUptimeProvider.getSeconds();
    double newRemoteUptime;
    try {
      newRemoteUptime = callable.call();
    } catch (Exception e) {
      log.error(e);
      throw new RosRuntimeException(e);
    }
    double latency = localUptimeProvider.getSeconds() - newLocalUptime;
    double latencyOffset = latency / 2;
    newLocalUptime += latencyOffset;
    return new UptimeCalculationResult(newLocalUptime, newRemoteUptime, latency);
  }

  /**
   * Returns the estimated local uptime in seconds for the given remote uptime.
   * 
   * @param remoteUptime
   *          the remote uptime to convert to local uptime
   * @return the estimated local uptime in seconds at the provided remote uptime
   */
  public double toLocalUptime(double remoteUptime) {
    double localOffset =
        (drift + errorReductionCoefficient) * (remoteUptime - predictedRemoteUptime);
    return localUptime + localOffset;
  }

  @VisibleForTesting
  double getDrift() {
    return drift;
  }

  @VisibleForTesting
  double getErrorReductionCoefficient() {
    return errorReductionCoefficient;
  }
}
