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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Before;
import org.junit.Test;

import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class CircularBlockingDequeTest {

  private ExecutorService executorService;

  @Before
  public void before() {
    executorService = Executors.newCachedThreadPool();
  }

  @Test
  public void testAddAndTake() throws InterruptedException {
    CircularBlockingDeque<String> deque = new CircularBlockingDeque<String>(10);
    String expectedString1 = "Hello, world!";
    String expectedString2 = "Goodbye, world!";
    deque.addLast(expectedString1);
    deque.addLast(expectedString2);
    assertEquals(expectedString1, deque.takeFirst());
    assertEquals(expectedString2, deque.takeFirst());
  }

  @Test
  public void testAddFirstAndTakeLast() throws InterruptedException {
    CircularBlockingDeque<String> deque = new CircularBlockingDeque<String>(10);
    String expectedString1 = "Hello, world!";
    String expectedString2 = "Goodbye, world!";
    deque.addLast(expectedString1);
    deque.addLast(expectedString2);
    assertEquals(expectedString1, deque.peekFirst());
    assertEquals(expectedString2, deque.takeLast());
    deque.addFirst(expectedString2);
    assertEquals(expectedString1, deque.peekLast());
    assertEquals(expectedString2, deque.takeFirst());
  }

  @Test
  public void testOverwrite() throws InterruptedException {
    CircularBlockingDeque<String> deque = new CircularBlockingDeque<String>(2);
    String expectedString = "Hello, world!";
    deque.addLast("overwritten");
    deque.addLast(expectedString);
    deque.addLast("foo");
    assertEquals(expectedString, deque.takeFirst());
  }

  @Test
  public void testIterator() throws InterruptedException {
    // We keep the queue short and throw in an unused element so that the deque
    // wraps around the backing array.
    CircularBlockingDeque<String> deque = new CircularBlockingDeque<String>(2);
    deque.addLast("unused");
    String expectedString1 = "Hello, world!";
    String expectedString2 = "Goodbye, world!";
    deque.addLast(expectedString1);
    deque.addLast(expectedString2);
    Iterator<String> iterator = deque.iterator();
    assertEquals(expectedString1, iterator.next());
    assertEquals(expectedString2, iterator.next());
    assertFalse(iterator.hasNext());
    try {
      iterator.next();
      fail();
    } catch (NoSuchElementException e) {
      // next() should throw an exception if there is no next element.
    }
    deque.takeFirst();
    iterator = deque.iterator();
    assertEquals(expectedString2, iterator.next());
    assertFalse(iterator.hasNext());
  }

  @Test
  public void testBlockingTake() throws InterruptedException {
    final CircularBlockingDeque<String> deque = new CircularBlockingDeque<String>(1);
    final String expectedString = "Hello, world!";
    final CountDownLatch latch = new CountDownLatch(1);
    executorService.execute(new Runnable() {
      @Override
      public void run() {
        try {
          assertEquals(expectedString, deque.takeFirst());
        } catch (InterruptedException e) {
          fail();
        }
        latch.countDown();
      }
    });
    // Sleep to ensure we're waiting on take().
    Thread.sleep(5);
    deque.addLast(expectedString);
    assertTrue(latch.await(1, TimeUnit.SECONDS));
  }
}
