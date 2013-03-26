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

package org.ros.internal.message.field;

import com.google.common.base.Preconditions;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.MessageBuffers;

import java.nio.ByteOrder;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ChannelBufferField extends Field {

  private final int size;

  private ChannelBuffer value;

  public static ChannelBufferField newVariable(FieldType type, String name, int size) {
    return new ChannelBufferField(type, name, size);
  }

  private ChannelBufferField(FieldType type, String name, int size) {
    super(type, name, false);
    this.size = size;
    value = MessageBuffers.dynamicBuffer();
  }

  @SuppressWarnings("unchecked")
  @Override
  public ChannelBuffer getValue() {
    // Return a defensive duplicate. Unlike with copy(), duplicated
    // ChannelBuffers share the same backing array, so this is relatively cheap.
    return value.duplicate();
  }

  @Override
  public void setValue(Object value) {
    Preconditions.checkArgument(((ChannelBuffer) value).order() == ByteOrder.LITTLE_ENDIAN);
    Preconditions.checkArgument(size < 0 || ((ChannelBuffer) value).readableBytes() == size);
    this.value = (ChannelBuffer) value;
  }

  @Override
  public void serialize(ChannelBuffer buffer) {
    if (size < 0) {
      buffer.writeInt(value.readableBytes());
    }
    // By specifying the start index and length we avoid modifying value's
    // indices and marks.
    buffer.writeBytes(value, 0, value.readableBytes());
  }

  @Override
  public void deserialize(ChannelBuffer buffer) {
    int currentSize = size;
    if (currentSize < 0) {
      currentSize = buffer.readInt();
    }
    value = buffer.readSlice(currentSize);
  }

  @Override
  public String getMd5String() {
    return String.format("%s %s\n", type, name);
  }

  @Override
  public String getJavaTypeName() {
    return "org.jboss.netty.buffer.ChannelBuffer";
  }

  @Override
  public String toString() {
    return "ChannelBufferField<" + type + ", " + name + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result = prime * result + ((value == null) ? 0 : value.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (!super.equals(obj))
      return false;
    if (getClass() != obj.getClass())
      return false;
    ChannelBufferField other = (ChannelBufferField) obj;
    if (value == null) {
      if (other.value != null)
        return false;
    } else if (!value.equals(other.value))
      return false;
    return true;
  }
}
