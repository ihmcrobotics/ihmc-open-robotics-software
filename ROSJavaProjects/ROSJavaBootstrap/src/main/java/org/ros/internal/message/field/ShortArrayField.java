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
public class ShortArrayField extends Field {

  private final int size;

  private short[] value;

  public static ShortArrayField newVariable(FieldType type, String name, int size) {
    return new ShortArrayField(type, name, size);
  }

  private ShortArrayField(FieldType type, String name, int size) {
    super(type, name, false);
    this.size = size;
    setValue(new short[Math.max(0, size)]);
  }

  @SuppressWarnings("unchecked")
  
  public short[] getValue() {
    return value;
  }

  
  public void setValue(Object value) {
    Preconditions.checkArgument(size < 0 || ((short[]) value).length == size);
    this.value = (short[]) value;
  }

  
  public void serialize(ChannelBuffer buffer) {
    if (size < 0) {
      buffer.writeInt(value.length);
    }
    for (short v : value) {
      type.serialize(v, buffer);
    }
  }

  
  public void deserialize(ChannelBuffer buffer) {
    int currentSize = size;
    if (currentSize < 0) {
      currentSize = buffer.readInt();
    }
    value = new short[currentSize];
    for (int i = 0; i < currentSize; i++) {
      value[i] = buffer.readShort();
    }
  }

  
  public String getMd5String() {
    return String.format("%s %s\n", type, name);
  }

  
  public String getJavaTypeName() {
    return type.getJavaTypeName() + "[]";
  }

  
  public String toString() {
    return "ShortArrayField<" + type + ", " + name + ">";
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
    ShortArrayField other = (ShortArrayField) obj;
    if (value == null) {
      if (other.value != null)
        return false;
    } else if (!Arrays.equals(value, other.value))
      return false;
    return true;
  }
}
