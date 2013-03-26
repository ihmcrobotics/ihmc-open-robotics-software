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

import com.google.common.base.Preconditions;

import org.jboss.netty.buffer.ChannelBuffer;

import java.util.ArrayList;
import java.util.List;

/**
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <T>
 *          the value type
 */
public class ListField<T> extends Field {

  private List<T> value;

  public static <T> ListField<T> newVariable(FieldType type, String name) {
    return new ListField<T>(type, name);
  }

  private ListField(FieldType type, String name) {
    super(type, name, false);
    value = new ArrayList<T>();
  }

  @SuppressWarnings("unchecked")
  @Override
  public List<T> getValue() {
    return value;
  }

  @SuppressWarnings("unchecked")
  @Override
  public void setValue(Object value) {
    Preconditions.checkNotNull(value);
    this.value = (List<T>) value;
  }

  @Override
  public void serialize(ChannelBuffer buffer) {
    buffer.writeInt(value.size());
    for (T v : value) {
      type.serialize(v, buffer);
    }
  }

  @Override
  public void deserialize(ChannelBuffer buffer) {
    value.clear();
    int size = buffer.readInt();
    for (int i = 0; i < size; i++) {
      value.add(type.<T>deserialize(buffer));
    }
  }

  @Override
  public String getMd5String() {
    return String.format("%s %s\n", type, name);
  }

  @Override
  public String getJavaTypeName() {
    return String.format("java.util.List<%s>", type.getJavaTypeName());
  }

  @Override
  public String toString() {
    return "ListField<" + type + ", " + name + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result = prime * result + ((value == null) ? 0 : value.hashCode());
    return result;
  }

  @SuppressWarnings("rawtypes")
  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (!super.equals(obj))
      return false;
    if (getClass() != obj.getClass())
      return false;
    ListField other = (ListField) obj;
    if (value == null) {
      if (other.value != null)
        return false;
    } else if (!value.equals(other.value))
      return false;
    return true;
  }
}
