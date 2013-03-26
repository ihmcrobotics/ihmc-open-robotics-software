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

import java.util.Iterator;
import java.util.NoSuchElementException;

/**
 * A deque that removes head or tail elements when the number of elements
 * exceeds the limit and blocks on {@link #takeFirst()} and {@link #takeLast()} when
 * there are no elements available.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class CircularBlockingDeque<T> implements Iterable<T> {

  private final T[] deque;
  private final Object mutex;

  /**
   * The maximum number of entries in the queue.
   */
  private final int limit;

  /**
   * Points to the next entry that will be returned by {@link #takeFirst()} unless
   * {@link #isEmpty()}.
   */
  private int start;

  /**
   * The number of entries in the queue.
   */
  private int length;

  /**
   * @param capacity
   *          the maximum number of elements allowed in the queue
   */
  @SuppressWarnings("unchecked")
  public CircularBlockingDeque(int capacity) {
    deque = (T[]) new Object[capacity];
    mutex = new Object();
    limit = capacity;
    start = 0;
    length = 0;
  }

  /**
   * Adds the specified entry to the tail of the queue, overwriting older
   * entries if necessary.
   * 
   * @param entry
   *          the entry to add
   * @return {@code true}
   */
  public boolean addLast(T entry) {
    synchronized (mutex) {
      deque[(start + length) % limit] = entry;
      if (length == limit) {
        start = (start + 1) % limit;
      } else {
        length++;
      }
      mutex.notify();
    }
    return true;
  }

  /**
   * Adds the specified entry to the tail of the queue, overwriting older
   * entries if necessary.
   * 
   * @param entry
   *          the entry to add
   * @return {@code true}
   */
  public boolean addFirst(T entry) {
    synchronized (mutex) {
      if (start - 1 < 0) {
        start = limit - 1;
      } else {
        start--;
      }
      deque[start] = entry;
      if (length < limit) {
        length++;
      }
      mutex.notify();
    }
    return true;
  }

  /**
   * Retrieves the head of the queue, blocking if necessary until an entry is
   * available.
   * 
   * @return the head of the queue
   * @throws InterruptedException
   */
  public T takeFirst() throws InterruptedException {
    T entry;
    synchronized (mutex) {
      while (true) {
        if (length > 0) {
          entry = deque[start];
          start = (start + 1) % limit;
          length--;
          break;
        }
        mutex.wait();
      }
    }
    return entry;
  }

  /**
   * Retrieves, but does not remove, the head of this queue, returning
   * {@code null} if this queue is empty.
   * 
   * @return the head of this queue, or {@code null} if this queue is empty
   */
  public T peekFirst() {
    synchronized (mutex) {
      if (length > 0) {
        return deque[start];
      }
      return null;
    }
  }

  /**
   * Retrieves the tail of the queue, blocking if necessary until an entry is
   * available.
   * 
   * @return the tail of the queue
   * @throws InterruptedException
   */
  public T takeLast() throws InterruptedException {
    T entry;
    synchronized (mutex) {
      while (true) {
        if (length > 0) {
          entry = deque[(start + length - 1) % limit];
          length--;
          break;
        }
        mutex.wait();
      }
    }
    return entry;
  }

  /**
   * Retrieves, but does not remove, the tail of this queue, returning
   * {@code null} if this queue is empty.
   * 
   * @return the tail of this queue, or {@code null} if this queue is empty
   */
  public T peekLast() {
    synchronized (mutex) {
      if (length > 0) {
        return deque[(start + length - 1) % limit];
      }
      return null;
    }
  }

  public boolean isEmpty() {
    return length == 0;
  }

  /**
   * Returns an iterator over the queue.
   * <p>
   * Note that this is not thread-safe and that {@link Iterator#remove()} is
   * unsupported.
   * 
   * @see java.lang.Iterable#iterator()
   */
  @Override
  public Iterator<T> iterator() {
    return new Iterator<T>() {
      int offset = 0;

      @Override
      public boolean hasNext() {
        return offset < length;
      }

      @Override
      public T next() {
        if (offset == length) {
          throw new NoSuchElementException();
        }
        T entry = deque[(start + offset) % limit];
        offset++;
        return entry;
      }

      @Override
      public void remove() {
        throw new UnsupportedOperationException();
      }
    };
  }
}
