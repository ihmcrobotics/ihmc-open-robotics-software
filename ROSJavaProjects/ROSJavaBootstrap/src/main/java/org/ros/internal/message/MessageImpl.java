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

package org.ros.internal.message;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.context.MessageContext;
import org.ros.internal.message.field.Field;
import org.ros.internal.message.field.MessageFieldType;
import org.ros.internal.message.field.MessageFields;
import org.ros.message.Duration;
import org.ros.message.MessageIdentifier;
import org.ros.message.Time;

import java.util.List;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
class MessageImpl implements RawMessage, GetInstance {

  private final MessageContext messageContext;
  private final MessageFields messageFields;

  public MessageImpl(MessageContext messageContext) {
    this.messageContext = messageContext;
    messageFields = new MessageFields(messageContext);
  }

  public MessageContext getMessageContext() {
    return messageContext;
  }

  public MessageFields getMessageFields() {
    return messageFields;
  }

  @Override
  public RawMessage toRawMessage() {
    return (RawMessage) this;
  }

  @Override
  public MessageIdentifier getIdentifier() {
    return messageContext.getMessageIdentifer();
  }

  @Override
  public String getType() {
    return messageContext.getType();
  }

  @Override
  public String getPackage() {
    return messageContext.getPackage();
  }

  @Override
  public String getName() {
    return messageContext.getName();
  }

  @Override
  public String getDefinition() {
    return messageContext.getDefinition();
  }

  @Override
  public List<Field> getFields() {
    return messageFields.getFields();
  }

  @Override
  public boolean getBool(String name) {
    return (Boolean) messageFields.getFieldValue(name);
  }

  @Override
  public boolean[] getBoolArray(String name) {
    return (boolean[]) messageFields.getFieldValue(name);
  }

  @Override
  public Duration getDuration(String name) {
    return (Duration) messageFields.getFieldValue(name);
  }

  @SuppressWarnings("unchecked")
  @Override
  public List<Duration> getDurationList(String name) {
    return (List<Duration>) messageFields.getFieldValue(name);
  }

  @Override
  public float getFloat32(String name) {
    return (Float) messageFields.getFieldValue(name);
  }

  @Override
  public float[] getFloat32Array(String name) {
    return (float[]) messageFields.getFieldValue(name);
  }

  @Override
  public double getFloat64(String name) {
    return (Double) messageFields.getFieldValue(name);
  }

  @Override
  public double[] getFloat64Array(String name) {
    return (double[]) messageFields.getFieldValue(name);
  }

  @Override
  public short getInt16(String name) {
    return (Short) messageFields.getFieldValue(name);
  }

  @Override
  public short[] getInt16Array(String name) {
    return (short[]) messageFields.getFieldValue(name);
  }

  @Override
  public int getInt32(String name) {
    return (Integer) messageFields.getFieldValue(name);
  }

  @Override
  public int[] getInt32Array(String name) {
    return (int[]) messageFields.getFieldValue(name);
  }

  @Override
  public long getInt64(String name) {
    return (Long) messageFields.getFieldValue(name);
  }

  @Override
  public long[] getInt64Array(String name) {
    return (long[]) messageFields.getFieldValue(name);
  }

  @Override
  public byte getInt8(String name) {
    return (Byte) messageFields.getFieldValue(name);
  }

  @Override
  public byte[] getInt8Array(String name) {
    return (byte[]) messageFields.getFieldValue(name);
  }

  @Override
  public <T extends Message> T getMessage(String name) {
    if (messageFields.getField(name).getType() instanceof MessageFieldType) {
      return messageFields.getField(name).<T>getValue();
    }
    throw new RosRuntimeException("Failed to access message field: " + name);
  }

  @Override
  public <T extends Message> List<T> getMessageList(String name) {
    if (messageFields.getField(name).getType() instanceof MessageFieldType) {
      return messageFields.getField(name).<List<T>>getValue();
    }
    throw new RosRuntimeException("Failed to access list field: " + name);
  }

  @Override
  public String getString(String name) {
    return (String) messageFields.getFieldValue(name);
  }

  @SuppressWarnings("unchecked")
  @Override
  public List<String> getStringList(String name) {
    return (List<String>) messageFields.getFieldValue(name);
  }

  @Override
  public Time getTime(String name) {
    return (Time) messageFields.getFieldValue(name);
  }

  @SuppressWarnings("unchecked")
  @Override
  public List<Time> getTimeList(String name) {
    return (List<Time>) messageFields.getFieldValue(name);
  }

  @Override
  public short getUInt16(String name) {
    return (Short) messageFields.getFieldValue(name);
  }

  @Override
  public short[] getUInt16Array(String name) {
    return (short[]) messageFields.getFieldValue(name);
  }

  @Override
  public int getUInt32(String name) {
    return (Integer) messageFields.getFieldValue(name);
  }

  @Override
  public int[] getUInt32Array(String name) {
    return (int[]) messageFields.getFieldValue(name);
  }

  @Override
  public long getUInt64(String name) {
    return (Long) messageFields.getFieldValue(name);
  }

  @Override
  public long[] getUInt64Array(String name) {
    return (long[]) messageFields.getFieldValue(name);
  }

  @Override
  public short getUInt8(String name) {
    return (Short) messageFields.getFieldValue(name);
  }

  @Override
  public short[] getUInt8Array(String name) {
    return (short[]) messageFields.getFieldValue(name);
  }

  @Override
  public void setBool(String name, boolean value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setBoolArray(String name, boolean[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setDurationList(String name, List<Duration> value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setDuration(String name, Duration value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setFloat32(String name, float value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setFloat32Array(String name, float[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setFloat64(String name, double value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setFloat64Array(String name, double[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setInt16(String name, short value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setInt16Array(String name, short[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setInt32(String name, int value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setInt32Array(String name, int[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setInt64(String name, long value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setInt64Array(String name, long[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setInt8(String name, byte value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setInt8Array(String name, byte[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setMessage(String name, Message value) {
    // TODO(damonkohler): Verify the type of the provided Message?
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setMessageList(String name, List<Message> value) {
    // TODO(damonkohler): Verify the type of all Messages in the provided list?
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setString(String name, String value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setStringList(String name, List<String> value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setTime(String name, Time value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setTimeList(String name, List<Time> value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setUInt16(String name, short value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setUInt16Array(String name, short[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setUInt32(String name, int value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setUInt32Array(String name, int[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setUInt64(String name, long value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setUInt64Array(String name, long[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setUInt8(String name, byte value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setUInt8Array(String name, byte[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public byte getByte(String name) {
    return (Byte) messageFields.getFieldValue(name);
  }

  @Override
  public short getChar(String name) {
    return (Short) messageFields.getFieldValue(name);
  }

  @Override
  public void setByte(String name, byte value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setChar(String name, short value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setByteArray(String name, byte[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public void setCharArray(String name, short[] value) {
    messageFields.setFieldValue(name, value);
  }

  @Override
  public byte[] getByteArray(String name) {
    return (byte[]) messageFields.getFieldValue(name);
  }

  @Override
  public short[] getCharArray(String name) {
    return (short[]) messageFields.getFieldValue(name);
  }
  
  @Override
  public ChannelBuffer getChannelBuffer(String name) {
    return (ChannelBuffer) messageFields.getFieldValue(name);
  }

  @Override
  public void setChannelBuffer(String name, ChannelBuffer value) {
    messageFields.setFieldValue(name, value);
  }
  
  @Override
  public Object getInstance() {
    return this;
  }

  @Override
  public String toString() {
    return String.format("MessageImpl<%s>", getType());
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((messageContext == null) ? 0 : messageContext.hashCode());
    result = prime * result + ((messageFields == null) ? 0 : messageFields.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (!(obj instanceof GetInstance))
      return false;
    obj = ((GetInstance) obj).getInstance();
    if (getClass() != obj.getClass())
      return false;
    MessageImpl other = (MessageImpl) obj;
    if (messageContext == null) {
      if (other.messageContext != null)
        return false;
    } else if (!messageContext.equals(other.messageContext))
      return false;
    if (messageFields == null) {
      if (other.messageFields != null)
        return false;
    } else if (!messageFields.equals(other.messageFields))
      return false;
    return true;
  }
}
