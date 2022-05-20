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

package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.topic.Publisher;

import java.util.concurrent.ScheduledExecutorService;

/**
 * Repeatedly send a message out on a given {@link Publisher}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class RepeatingPublisher<MessageType> {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(RepeatingPublisher.class);

  private final Publisher<MessageType> publisher;
  private final MessageType message;
  private final int frequency;
  private final RepeatingPublisherLoop runnable;

  /**
   * Executor used to run the {@link RepeatingPublisherLoop}.
   */
  private final ScheduledExecutorService executorService;

  private final class RepeatingPublisherLoop extends CancellableLoop {
    @Override
    public void loop() throws InterruptedException {
      publisher.publish(message);
      if (DEBUG) {
        log.info(String.format("Published message %s to publisher %s ", message, publisher));
      }
      Thread.sleep((long) (1000.0d / frequency));
    }
  }

  /**
   * @param publisher
   * @param message
   * @param frequency
   *          the frequency of publication in Hz
   */
  public RepeatingPublisher(Publisher<MessageType> publisher, MessageType message, int frequency,
      ScheduledExecutorService executorService) {
    this.publisher = publisher;
    this.message = message;
    this.frequency = frequency;
    this.executorService = executorService;
    runnable = new RepeatingPublisherLoop();
  }

  public void start() {
    Preconditions.checkState(!runnable.isRunning());
    executorService.execute(runnable);
  }

  public void cancel() {
    Preconditions.checkState(runnable.isRunning());
    runnable.cancel();
  }
}
