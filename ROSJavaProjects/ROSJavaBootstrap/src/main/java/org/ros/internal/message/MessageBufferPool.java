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

package org.ros.internal.message;

import org.apache.commons.pool.ObjectPool;
import org.apache.commons.pool.PoolableObjectFactory;
import org.apache.commons.pool.impl.StackObjectPool;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.exception.RosRuntimeException;

/**
 * A pool of {@link ChannelBuffer}s for serializing and deserializing messages.
 * <p>
 * By contract, {@link ChannelBuffer}s provided by {@link #acquire()} must be
 * returned using {@link #release(ChannelBuffer)}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageBufferPool {

  private final ObjectPool<ChannelBuffer> pool;

  public MessageBufferPool() {
    pool = new StackObjectPool<ChannelBuffer>(new PoolableObjectFactory<ChannelBuffer>() {
      @Override
      public ChannelBuffer makeObject() throws Exception {
        return MessageBuffers.dynamicBuffer();
      }

      @Override
      public void destroyObject(ChannelBuffer channelBuffer) throws Exception {
      }

      @Override
      public boolean validateObject(ChannelBuffer channelBuffer) {
        return true;
      }

      @Override
      public void activateObject(ChannelBuffer channelBuffer) throws Exception {
      }

      @Override
      public void passivateObject(ChannelBuffer channelBuffer) throws Exception {
        channelBuffer.clear();
      }
    });
  }

  /**
   * Acquired {@link ChannelBuffer}s must be returned using
   * {@link #release(ChannelBuffer)}.
   * 
   * @return an unused {@link ChannelBuffer}
   */
  public ChannelBuffer acquire() {
    try {
      return pool.borrowObject();
    } catch (Exception e) {
      throw new RosRuntimeException(e);
    }
  }

  /**
   * Release a previously acquired {@link ChannelBuffer}.
   * 
   * @param channelBuffer
   *          the {@link ChannelBuffer} to release
   */
  public void release(ChannelBuffer channelBuffer) {
    try {
      pool.returnObject(channelBuffer);
    } catch (Exception e) {
      throw new RosRuntimeException(e);
    }
  }
}
