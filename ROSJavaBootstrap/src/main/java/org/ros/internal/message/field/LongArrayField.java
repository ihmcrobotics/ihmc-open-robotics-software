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

package org.ros.internal.message.field;

import java.util.Arrays;

import org.jboss.netty.buffer.ChannelBuffer;

import com.google.common.base.Preconditions;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class LongArrayField extends Field {

  private final int size;

  private long[] value;

  public static LongArrayField newVariable(FieldType type, String name, int size) {
    Preconditions.checkArgument(type.equals(PrimitiveFieldType.UINT32)
        || type.equals(PrimitiveFieldType.INT64) || type.equals(PrimitiveFieldType.UINT64));
    return new LongArrayField(type, name, size);
  }

  private LongArrayField(FieldType type, String name, int size) {
    super(type, name, false);
    this.size = size;
    setValue(new long[Math.max(0, size)]);
  }

  @SuppressWarnings("unchecked")
  
  public long[] getValue() {
    return value;
  }

  
  public void setValue(Object value) {
    Preconditions.checkArgument(size < 0 || ((long[]) value).length == size);
    this.value = (long[]) value;
  }

  
  public void serialize(ChannelBuffer buffer) {
    if (size < 0) {
      buffer.writeInt(value.length);
    }
    for (long v : value) {
      type.serialize(v, buffer);
    }
  }

  
  public void deserialize(ChannelBuffer buffer) {
    int currentSize = size;
    if (currentSize < 0) {
      currentSize = buffer.readInt();
    }
    value = new long[currentSize];
    for (int i = 0; i < currentSize; i++) {
      value[i] = buffer.readLong();
    }
  }

  
  public String getMd5String() {
    return String.format("%s %s\n", type, name);
  }

  
  public String getJavaTypeName() {
    return type.getJavaTypeName() + "[]";
  }

  
  public String toString() {
    return "LongArrayField<" + type + ", " + name + ">";
  }

  
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result = prime * result + ((value == null) ? 0 : value.hashCode());
    return result;
  }

  
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (!super.equals(obj))
      return false;
    if (getClass() != obj.getClass())
      return false;
    LongArrayField other = (LongArrayField) obj;
    if (value == null) {
      if (other.value != null)
        return false;
    } else if (!Arrays.equals(value, other.value))
      return false;
    return true;
  }
}
