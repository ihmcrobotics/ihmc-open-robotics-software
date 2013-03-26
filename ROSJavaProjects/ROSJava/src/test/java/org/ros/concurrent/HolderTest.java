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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class HolderTest {

  @Test
  public void testHolderFailsWithNoValue() {
    Holder<String> holder = Holder.newEmpty();
    try {
      holder.get();
      fail();
    } catch (Exception e) {
      // Holder.get() should fail if no value has been stored.
    }
  }

  @Test
  public void testHolderFailsIfSetTwice() {
    Holder<String> holder = Holder.newEmpty();
    holder.set("Hello, world!");
    try {
      holder.set("Goodbye, world!");
      fail();
    } catch (Exception e) {
      // Holder.set() should fail if a value has already been stored.
    }
  }
  
  @Test
  public void testHolderEquality() {
    Holder<String> holder1 = Holder.newEmpty();
    Holder<String> holder2 = Holder.newEmpty();
    assertTrue(holder1.equals(holder1));
    assertFalse(holder1.equals(holder2));
  }

  @Test
  public void testHolderAwait() throws InterruptedException {
    final Holder<String> holder = Holder.newEmpty();
    final String message = "Hello, world!";
    ExecutorService executorService = Executors.newCachedThreadPool();
    executorService.execute(new Runnable() {
      @Override
      public void run() {
        holder.set(message);
      }
    });
    assertTrue(holder.await(1, TimeUnit.SECONDS));
    assertEquals(message, holder.get());
  }
}
