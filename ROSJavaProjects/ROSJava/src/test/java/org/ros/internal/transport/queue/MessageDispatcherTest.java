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

package org.ros.internal.transport.queue;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Before;
import org.junit.Test;
import org.ros.concurrent.CircularBlockingDeque;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import std_msgs.Int32;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageDispatcherTest {

  private static final int QUEUE_CAPACITY = 128;

  private ExecutorService executorService;
  private CircularBlockingDeque<LazyMessage<std_msgs.Int32>> lazyMessages;
  private MessageFactory messageFactory;

  @Before
  public void before() {
    executorService = Executors.newCachedThreadPool();
    lazyMessages = new CircularBlockingDeque<LazyMessage<std_msgs.Int32>>(128);
    messageFactory = new DefaultMessageFactory(new MessageDefinitionReflectionProvider());
  }

  @Test
  public void testMessageOrder() throws InterruptedException {
    int numberOfMessages = 100;
    final CountDownLatch latch = new CountDownLatch(numberOfMessages);

    MessageDispatcher<std_msgs.Int32> messageDispatcher =
        new MessageDispatcher<std_msgs.Int32>(lazyMessages, executorService);
    messageDispatcher.addListener(new MessageListener<std_msgs.Int32>() {
      private AtomicInteger count = new AtomicInteger();

      @Override
      public void onNewMessage(Int32 message) {
        if (this.count.compareAndSet(message.getData(), message.getData() + 1)) {
          latch.countDown();
        } else {
          fail(String.format("Expected message data not equal to actual data: %d != %d",
              count.get(), message.getData()));
        }
        try {
          // Sleeping allows the queue to fill up a bit by slowing down the
          // consumer.
          Thread.sleep(5);
        } catch (InterruptedException e) {
        }
      }
    }, QUEUE_CAPACITY);
    executorService.execute(messageDispatcher);

    for (int i = 0; i < numberOfMessages; i++) {
      final int count = i;
      std_msgs.Int32 message = messageFactory.newFromType(std_msgs.Int32._TYPE);
      message.setData(count);
      lazyMessages.addLast(new LazyMessage<std_msgs.Int32>(message));
    }

    assertTrue(latch.await(1, TimeUnit.SECONDS));
  }
}
