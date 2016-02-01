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

import org.jboss.netty.buffer.ChannelBuffer;

import com.google.common.base.Preconditions;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
class ValueField<T> extends Field {

  private T value;

  static <T> ValueField<T> newConstant(FieldType type, String name, T value) {
    return new ValueField<T>(type, name, value, true);
  }

  static <T> ValueField<T> newVariable(FieldType type, String name) {
    return new ValueField<T>(type, name, null, false);
  }

  private ValueField(FieldType type, String name, T value, boolean isConstant) {
    super(type, name, isConstant);
    this.value = value;
  }

  @SuppressWarnings("unchecked")
  
  public T getValue() {
    if (value == null) {
      setValue(type.getDefaultValue());
    }
    return value;
  }

  @SuppressWarnings("unchecked")
  
  public void setValue(Object value) {
    Preconditions.checkNotNull(value);
    Preconditions.checkState(!isConstant);
    this.value = (T) value;
  }

  
  public void serialize(ChannelBuffer buffer) {
    type.serialize(getValue(), buffer);
  }

  
  public void deserialize(ChannelBuffer buffer) {
    Preconditions.checkState(!isConstant);
    setValue(type.<T>deserialize(buffer));
  }

  
  public String getMd5String() {
    return String.format("%s %s\n", type, name);
  }

  
  public String getJavaTypeName() {
    return type.getJavaTypeName();
  }

  
  public String toString() {
    return "ValueField<" + type + ", " + name + ">";
  }

  
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result = prime * result + ((getValue() == null) ? 0 : getValue().hashCode());
    return result;
  }

  
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (!super.equals(obj))
      return false;
    if (getClass() != obj.getClass())
      return false;
    Field other = (Field) obj;
    if (getValue() == null) {
      if (other.getValue() != null)
        return false;
    } else if (!getValue().equals(other.getValue()))
      return false;
    return true;
  }
}
