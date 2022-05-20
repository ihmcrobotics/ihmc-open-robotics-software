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

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * A mutable object that may contain a value to another object. It is modifiable
 * exactly once and must hold a non-null value when the value is inspected.
 * 
 * <p>
 * {@link Holder}s are intended for receiving a result from an anonymous class.
 * 
 * <p>
 * Note that {@link Holder} is not thread safe. For a thread safe
 * implementation, use {@link AtomicReference}. Also note that two different
 * {@link Holder} instances are never considered equal.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class Holder<T> {

  private final CountDownLatch latch;

  private T value;

  public static <T> Holder<T> newEmpty() {
    return new Holder<T>();
  }

  private Holder() {
    latch = new CountDownLatch(1);
    value = null;
  }

  public T set(T value) {
    Preconditions.checkState(this.value == null);
    this.value = value;
    latch.countDown();
    return value;
  }

  public T get() {
    Preconditions.checkNotNull(value);
    return value;
  }
  
  public void await() throws InterruptedException {
    latch.await();
  }

  public boolean await(long timeout, TimeUnit unit) throws InterruptedException {
    return latch.await(timeout, unit);
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    return false;
  }
}
