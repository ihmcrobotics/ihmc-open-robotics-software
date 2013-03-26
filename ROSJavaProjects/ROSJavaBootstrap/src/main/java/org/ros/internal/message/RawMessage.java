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
import org.ros.internal.message.field.Field;
import org.ros.message.Duration;
import org.ros.message.MessageIdentifier;
import org.ros.message.Time;

import java.util.List;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface RawMessage extends Message {

  boolean getBool(String name);

  boolean[] getBoolArray(String name);

  /**
   * @deprecated replaced by {@link #getInt8(String)}
   */
  byte getByte(String name);

  /**
   * @deprecated replaced by {@link #getInt8Array(String)}
   */
  byte[] getByteArray(String name);

  /**
   * @deprecated replaced by {@link #getUInt8(String)}
   */
  short getChar(String name);

  /**
   * @deprecated replaced by {@link #getUInt8Array(String)}
   */
  short[] getCharArray(String name);

  String getDefinition();

  Duration getDuration(String name);

  List<Duration> getDurationList(String name);

  List<Field> getFields();

  float getFloat32(String name);

  float[] getFloat32Array(String name);

  double getFloat64(String name);

  double[] getFloat64Array(String name);

  MessageIdentifier getIdentifier();

  short getInt16(String name);

  short[] getInt16Array(String name);

  int getInt32(String name);

  int[] getInt32Array(String name);

  long getInt64(String name);

  long[] getInt64Array(String name);

  byte getInt8(String name);

  byte[] getInt8Array(String name);

  <T extends Message> T getMessage(String name);

  <T extends Message> List<T> getMessageList(String name);

  String getName();

  String getPackage();

  String getString(String name);

  List<String> getStringList(String name);

  Time getTime(String name);

  List<Time> getTimeList(String name);

  String getType();

  short getUInt16(String name);

  short[] getUInt16Array(String name);

  int getUInt32(String name);

  int[] getUInt32Array(String name);

  long getUInt64(String name);

  long[] getUInt64Array(String name);

  short getUInt8(String name);

  short[] getUInt8Array(String name);

  void setBool(String name, boolean value);

  void setBoolArray(String name, boolean[] value);

  /**
   * @deprecated replaced by {@link #setInt8(String, byte)}
   */
  void setByte(String name, byte value);

  /**
   * @deprecated replaced by {@link #setInt8Array(String, byte[])}
   */
  void setByteArray(String name, byte[] value);

  /**
   * @deprecated replaced by {@link #setUInt8(String, byte)}
   */
  void setChar(String name, short value);

  /**
   * @deprecated replaced by {@link #setUInt8Array(String, byte[])}
   */
  void setCharArray(String name, short[] value);

  void setDuration(String name, Duration value);

  void setDurationList(String name, List<Duration> value);

  void setFloat32(String name, float value);

  void setFloat32Array(String name, float[] value);

  void setFloat64(String name, double value);

  void setFloat64Array(String name, double[] value);

  void setInt16(String name, short value);

  void setInt16Array(String name, short[] value);

  void setInt32(String name, int value);

  void setInt32Array(String name, int[] value);

  void setInt64(String name, long value);

  void setInt64Array(String name, long[] value);

  void setInt8(String name, byte value);

  void setInt8Array(String name, byte[] value);

  void setMessage(String name, Message value);

  void setMessageList(String name, List<Message> value);

  void setString(String name, String value);

  void setStringList(String name, List<String> value);

  void setTime(String name, Time value);

  void setTimeList(String name, List<Time> value);

  void setUInt16(String name, short value);

  void setUInt16Array(String name, short[] value);

  void setUInt32(String name, int value);

  void setUInt32Array(String name, int[] value);

  void setUInt64(String name, long value);

  void setUInt64Array(String name, long[] value);

  void setUInt8(String name, byte value);

  void setUInt8Array(String name, byte[] value);

  void setChannelBuffer(String name, ChannelBuffer value);

  ChannelBuffer getChannelBuffer(String name);
}
