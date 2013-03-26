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
import com.google.common.collect.ImmutableSet;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Duration;
import org.ros.message.Time;

import java.nio.ByteBuffer;
import java.nio.charset.Charset;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public enum PrimitiveFieldType implements FieldType {

  BOOL {
    @SuppressWarnings("unchecked")
    @Override
    public Boolean getDefaultValue() {
      return Boolean.FALSE;
    }

    @Override
    public BooleanArrayField newVariableList(String name, int size) {
      return BooleanArrayField.newVariable(name, size);
    }

    @Override
    public int getSerializedSize() {
      return 1;
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Boolean);
      buffer.writeByte((byte) ((Boolean) value ? 1 : 0));
    }

    @SuppressWarnings("unchecked")
    @Override
    public Boolean deserialize(ChannelBuffer buffer) {
      return buffer.readByte() == 1;
    }

    @SuppressWarnings("unchecked")
    @Override
    public Boolean parseFromString(String value) {
      return value.equals("1");
    }

    @Override
    public String getJavaTypeName() {
      return "boolean";
    }
  },
  INT8 {
    @SuppressWarnings("unchecked")
    @Override
    public Byte getDefaultValue() {
      return Byte.valueOf((byte) 0);
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ChannelBufferField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return 1;
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Byte);
      buffer.writeByte((Byte) value);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Byte deserialize(ChannelBuffer buffer) {
      return buffer.readByte();
    }

    @SuppressWarnings("unchecked")
    @Override
    public Byte parseFromString(String value) {
      return Byte.parseByte(value);
    }

    @Override
    public String getJavaTypeName() {
      return "byte";
    }
  },
  /**
   * @deprecated replaced by {@link PrimitiveFieldType#INT8}
   */
  BYTE {
    @SuppressWarnings("unchecked")
    @Override
    public Byte getDefaultValue() {
      return INT8.getDefaultValue();
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ChannelBufferField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return INT8.getSerializedSize();
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      INT8.serialize(value, buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Byte deserialize(ChannelBuffer buffer) {
      return INT8.deserialize(buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Byte parseFromString(String value) {
      return INT8.parseFromString(value);
    }

    @Override
    public String getJavaTypeName() {
      return INT8.getJavaTypeName();
    }
  },
  UINT8 {
    @SuppressWarnings("unchecked")
    @Override
    public Byte getDefaultValue() {
      return INT8.getDefaultValue();
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ChannelBufferField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return INT8.getSerializedSize();
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      INT8.serialize(value, buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Byte deserialize(ChannelBuffer buffer) {
      return INT8.deserialize(buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Byte parseFromString(String value) {
      return (byte) Short.parseShort(value);
    }

    @Override
    public String getJavaTypeName() {
      return INT8.getJavaTypeName();
    }
  },
  /**
   * @deprecated replaced by {@link PrimitiveFieldType#UINT8}
   */
  CHAR {
    @SuppressWarnings("unchecked")
    @Override
    public Byte getDefaultValue() {
      return UINT8.getDefaultValue();
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ChannelBufferField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return UINT8.getSerializedSize();
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      UINT8.serialize(value, buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Byte deserialize(ChannelBuffer buffer) {
      return UINT8.deserialize(buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Byte parseFromString(String value) {
      return UINT8.parseFromString(value);
    }

    @Override
    public String getJavaTypeName() {
      return UINT8.getJavaTypeName();
    }
  },
  INT16 {
    @SuppressWarnings("unchecked")
    @Override
    public Short getDefaultValue() {
      return Short.valueOf((short) 0);
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ShortArrayField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return 2;
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Short);
      buffer.writeShort((Short) value);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Short deserialize(ChannelBuffer buffer) {
      return buffer.readShort();
    }

    @SuppressWarnings("unchecked")
    @Override
    public Short parseFromString(String value) {
      return Short.parseShort(value);
    }

    @Override
    public String getJavaTypeName() {
      return "short";
    }
  },
  UINT16 {
    @SuppressWarnings("unchecked")
    @Override
    public Short getDefaultValue() {
      return INT16.getDefaultValue();
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ShortArrayField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return INT16.getSerializedSize();
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      INT16.serialize(value, buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Short deserialize(ChannelBuffer buffer) {
      return INT16.deserialize(buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Short parseFromString(String value) {
      return (short) Integer.parseInt(value);
    }

    @Override
    public String getJavaTypeName() {
      return INT16.getJavaTypeName();
    }
  },
  INT32 {
    @SuppressWarnings("unchecked")
    @Override
    public Integer getDefaultValue() {
      return Integer.valueOf(0);
    }

    @Override
    public Field newVariableList(String name, int size) {
      return IntegerArrayField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return 4;
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Integer);
      buffer.writeInt((Integer) value);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Integer deserialize(ChannelBuffer buffer) {
      return buffer.readInt();
    }

    @SuppressWarnings("unchecked")
    @Override
    public Integer parseFromString(String value) {
      return Integer.parseInt(value);
    }

    @Override
    public String getJavaTypeName() {
      return "int";
    }
  },
  UINT32 {
    @SuppressWarnings("unchecked")
    @Override
    public Integer getDefaultValue() {
      return INT32.getDefaultValue();
    }

    @Override
    public Field newVariableList(String name, int size) {
      return IntegerArrayField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return INT32.getSerializedSize();
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      INT32.serialize(value, buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Integer deserialize(ChannelBuffer buffer) {
      return INT32.deserialize(buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Integer parseFromString(String value) {
      return (int) Long.parseLong(value);
    }

    @Override
    public String getJavaTypeName() {
      return INT32.getJavaTypeName();
    }
  },
  INT64 {
    @SuppressWarnings("unchecked")
    @Override
    public Long getDefaultValue() {
      return Long.valueOf(0);
    }

    @Override
    public Field newVariableList(String name, int size) {
      return LongArrayField.newVariable(this, name, size);
    }

    @Override
    public int getSerializedSize() {
      return 8;
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Long);
      buffer.writeLong((Long) value);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Long deserialize(ChannelBuffer buffer) {
      return buffer.readLong();
    }

    @SuppressWarnings("unchecked")
    @Override
    public Long parseFromString(String value) {
      return Long.parseLong(value);
    }

    @Override
    public String getJavaTypeName() {
      return "long";
    }
  },
  UINT64 {
    @SuppressWarnings("unchecked")
    @Override
    public Long getDefaultValue() {
      return INT64.getDefaultValue();
    }

    @Override
    public Field newVariableList(String name, int size) {
      return INT64.newVariableList(name, size);
    }

    @Override
    public int getSerializedSize() {
      return INT64.getSerializedSize();
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      INT64.serialize(value, buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Long deserialize(ChannelBuffer buffer) {
      return INT64.deserialize(buffer);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Long parseFromString(String value) {
      return INT64.parseFromString(value);
    }

    @Override
    public String getJavaTypeName() {
      return INT64.getJavaTypeName();
    }
  },
  FLOAT32 {
    @SuppressWarnings("unchecked")
    @Override
    public Float getDefaultValue() {
      return Float.valueOf(0);
    }

    @Override
    public Field newVariableList(String name, int size) {
      return FloatArrayField.newVariable(name, size);
    }

    @Override
    public int getSerializedSize() {
      return 4;
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Float);
      buffer.writeFloat((Float) value);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Float deserialize(ChannelBuffer buffer) {
      return buffer.readFloat();
    }

    @SuppressWarnings("unchecked")
    @Override
    public Float parseFromString(String value) {
      return Float.parseFloat(value);
    }

    @Override
    public String getJavaTypeName() {
      return "float";
    }
  },
  FLOAT64 {
    @SuppressWarnings("unchecked")
    @Override
    public Double getDefaultValue() {
      return Double.valueOf(0);
    }

    @Override
    public int getSerializedSize() {
      return 8;
    }

    @Override
    public Field newVariableList(String name, int size) {
      return DoubleArrayField.newVariable(name, size);
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Double);
      buffer.writeDouble((Double) value);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Double deserialize(ChannelBuffer buffer) {
      return buffer.readDouble();
    }

    @SuppressWarnings("unchecked")
    @Override
    public Double parseFromString(String value) {
      return Double.parseDouble(value);
    }

    @Override
    public String getJavaTypeName() {
      return "double";
    }
  },
  STRING {
    @SuppressWarnings("unchecked")
    @Override
    public String getDefaultValue() {
      return "";
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ListField.newVariable(this, name);
    }

    @Override
    public int getSerializedSize() {
      throw new UnsupportedOperationException();
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof String);
      byte[] bytes = ((String) value).getBytes();
      buffer.writeInt(bytes.length);
      buffer.writeBytes(bytes);
    }

    @SuppressWarnings("unchecked")
    @Override
    public String deserialize(ChannelBuffer buffer) {
      int length = buffer.readInt();
      ByteBuffer stringBuffer = buffer.readSlice(length).toByteBuffer();
      return Charset.forName("US-ASCII").decode(stringBuffer).toString();
    }

    @SuppressWarnings("unchecked")
    @Override
    public String parseFromString(String value) {
      return value;
    }

    @Override
    public String getJavaTypeName() {
      return "java.lang.String";
    }
  },
  TIME {
    @SuppressWarnings("unchecked")
    @Override
    public Time getDefaultValue() {
      return new Time();
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ListField.newVariable(this, name);
    }

    @Override
    public int getSerializedSize() {
      return 8;
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Time);
      buffer.writeInt(((Time) value).secs);
      buffer.writeInt(((Time) value).nsecs);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Time deserialize(ChannelBuffer buffer) {
      return new Time(buffer.readInt(), buffer.readInt());
    }

    @SuppressWarnings("unchecked")
    @Override
    public Void parseFromString(String value) {
      throw new UnsupportedOperationException();
    }

    @Override
    public String getJavaTypeName() {
      return Time.class.getName();
    }
  },
  DURATION {
    @SuppressWarnings("unchecked")
    @Override
    public Duration getDefaultValue() {
      return new Duration();
    }

    @Override
    public Field newVariableList(String name, int size) {
      return ListField.newVariable(this, name);
    }

    @Override
    public int getSerializedSize() {
      return 8;
    }

    @Override
    public <T> void serialize(T value, ChannelBuffer buffer) {
      Preconditions.checkArgument(value instanceof Duration);
      buffer.writeInt(((Duration) value).secs);
      buffer.writeInt(((Duration) value).nsecs);
    }

    @SuppressWarnings("unchecked")
    @Override
    public Duration deserialize(ChannelBuffer buffer) {
      return new Duration(buffer.readInt(), buffer.readInt());
    }

    @SuppressWarnings("unchecked")
    @Override
    public Void parseFromString(String value) {
      throw new UnsupportedOperationException();
    }

    @Override
    public String getJavaTypeName() {
      return Duration.class.getName();
    }
  };

  private static final ImmutableSet<String> TYPE_NAMES;

  static {
    ImmutableSet.Builder<String> builder = ImmutableSet.<String>builder();
    for (PrimitiveFieldType type : values()) {
      builder.add(type.getName());
    }
    TYPE_NAMES = builder.build();
  }

  public static boolean existsFor(String name) {
    return TYPE_NAMES.contains(name);
  }

  @Override
  public Field newVariableValue(String name) {
    return ValueField.newVariable(this, name);
  }

  @Override
  public <T> Field newConstantValue(String name, T value) {
    return ValueField.newConstant(this, name, value);
  }

  @Override
  public String getName() {
    return toString().toLowerCase();
  }

  @Override
  public String getMd5String() {
    return getName();
  }
}
