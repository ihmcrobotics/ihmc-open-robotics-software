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

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface FieldType {

  public <T> T getDefaultValue();

  public String getName();

  public <T> T parseFromString(String value);

  public String getMd5String();

  public String getJavaTypeName();

  /**
   * @return the serialized size of this {@link FieldType} in bytes
   */
  public int getSerializedSize();

  public <T> void serialize(T value, ChannelBuffer buffer);

  public <T> T deserialize(ChannelBuffer buffer);

  public Field newVariableValue(String name);

  public Field newVariableList(String name, int size);

  public <T> Field newConstantValue(String name, T value);
}
