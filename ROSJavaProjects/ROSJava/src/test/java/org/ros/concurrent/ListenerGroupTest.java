/*
 * Copyright (C) 2012 Google Inc.
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

import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ListenerGroupTest {

  private ExecutorService executorService;
  private ListenerGroup<Runnable> listenerGroup;

  @Before
  public void before() {
    executorService = Executors.newCachedThreadPool();
    listenerGroup = new ListenerGroup<Runnable>(executorService);
  }

  @Test
  public void testOneListenerMultipleSignals() throws InterruptedException {
    int numberOfSignals = 10;
    final CountDownLatch latch = new CountDownLatch(numberOfSignals);
    listenerGroup.add(new Runnable() {
      @Override
      public void run() {
        latch.countDown();
      }
    });
    for (int i = 0; i < numberOfSignals; i++) {
      listenerGroup.signal(new SignalRunnable<Runnable>() {
        @Override
        public void run(Runnable listener) {
          listener.run();
        }
      });
    }
    assertTrue(latch.await(1, TimeUnit.SECONDS));
  }

  @Test
  public void testMultipleListenersMultipleSignals() throws InterruptedException {
    int numberOfSignals = 10;
    final CountDownLatch latch1 = new CountDownLatch(numberOfSignals);
    final CountDownLatch latch2 = new CountDownLatch(numberOfSignals);
    listenerGroup.add(new Runnable() {
      @Override
      public void run() {
        latch1.countDown();
      }
    });
    listenerGroup.add(new Runnable() {
      @Override
      public void run() {
        latch2.countDown();
      }
    });
    for (int i = 0; i < numberOfSignals; i++) {
      listenerGroup.signal(new SignalRunnable<Runnable>() {
        @Override
        public void run(Runnable listener) {
          listener.run();
        }
      });
    }
    assertTrue(latch1.await(1, TimeUnit.SECONDS));
    assertTrue(latch2.await(1, TimeUnit.SECONDS));
  }

  private interface CountingListener {
    void run(int count);
  }

  @Test
  public void testSignalOrder() throws InterruptedException {
    int numberOfSignals = 100;
    final CountDownLatch latch = new CountDownLatch(numberOfSignals);

    ListenerGroup<CountingListener> listenerGroup =
        new ListenerGroup<CountingListener>(executorService);
    listenerGroup.add(new CountingListener() {
      private AtomicInteger count = new AtomicInteger();

      @Override
      public void run(int count) {
        if (this.count.compareAndSet(count, count + 1)) {
          latch.countDown();
        }
        try {
          // Sleeping allows the queue to fill up a bit by slowing down the
          // consumer.
          Thread.sleep(5);
        } catch (InterruptedException e) {
        }
      }
    });

    for (int i = 0; i < numberOfSignals; i++) {
      final int count = i;
      listenerGroup.signal(new SignalRunnable<CountingListener>() {
        @Override
        public void run(CountingListener listener) {
          listener.run(count);
        }
      });
    }

    assertTrue(latch.await(1, TimeUnit.SECONDS));
  }
}
