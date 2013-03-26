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
public abstract class Field {

  protected final FieldType type;
  protected final String name;
  protected final boolean isConstant;

  protected Field(FieldType type, String name, boolean isConstant) {
    this.name = name;
    this.type = type;
    this.isConstant = isConstant;
  }

  /**
   * @return the name
   */
  public String getName() {
    return name;
  }

  /**
   * @return the type
   */
  public FieldType getType() {
    return type;
  }

  /**
   * @return <code>true</code> if this {@link ListField} represents a constant
   */
  public boolean isConstant() {
    return isConstant;
  }

  /**
   * @return the textual representation of this field used for computing the MD5
   *         of a message definition
   */
  public String getMd5String() {
    if (isConstant()) {
      return String.format("%s %s=%s\n", getType().getMd5String(), getName(), getValue());
    }
    return String.format("%s %s\n", getType().getMd5String(), getName());
  }

  public abstract void serialize(ChannelBuffer buffer);

  public abstract void deserialize(ChannelBuffer buffer);

  public abstract <T> T getValue();

  // TODO(damonkohler): Why not make Field generic?
  public abstract void setValue(Object value);

  public abstract String getJavaTypeName();

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + (isConstant ? 1231 : 1237);
    result = prime * result + ((name == null) ? 0 : name.hashCode());
    result = prime * result + ((type == null) ? 0 : type.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    Field other = (Field) obj;
    if (isConstant != other.isConstant)
      return false;
    if (name == null) {
      if (other.name != null)
        return false;
    } else if (!name.equals(other.name))
      return false;
    if (type == null) {
      if (other.type != null)
        return false;
    } else if (!type.equals(other.type))
      return false;
    return true;
  }
}
