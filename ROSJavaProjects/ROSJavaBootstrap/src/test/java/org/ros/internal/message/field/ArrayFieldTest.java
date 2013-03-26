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

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.ros.internal.message.MessageBuffers;

import org.jboss.netty.buffer.ChannelBuffer;
import org.junit.Test;

/**
 * The following unit tests were created by inspecting the serialization of
 * array fields using the ROS Python client library.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ArrayFieldTest {

  @Test
  public void testBooleanArrayFieldVariableSize() {
    BooleanArrayField field = BooleanArrayField.newVariable("foo", -1);
    boolean[] value = new boolean[] { true, false, true, false };
    field.setValue(value);
    assertEquals(PrimitiveFieldType.BOOL, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 4, 0, 0, 0, 1, 0, 1, 0 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testBooleanArrayFieldFixedSize() {
    BooleanArrayField field = BooleanArrayField.newVariable("foo", 4);
    field.setValue(new boolean[] { true, false, true, false });
    assertEquals(PrimitiveFieldType.BOOL, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 1, 0, 1, 0 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @SuppressWarnings("deprecation")
  @Test
  public void testByteArrayFieldVariableSize() {
    testByteArrayFieldVariableSize(PrimitiveFieldType.INT8);
    testByteArrayFieldVariableSize(PrimitiveFieldType.BYTE);
    testByteArrayFieldVariableSize(PrimitiveFieldType.UINT8);
    testByteArrayFieldVariableSize(PrimitiveFieldType.CHAR);
  }

  private void testByteArrayFieldVariableSize(FieldType type) {
    ByteArrayField field = ByteArrayField.newVariable(type, "foo", -1);
    field.setValue(new byte[] { 1, 2, 3, 4 });
    assertEquals(type, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 4, 0, 0, 0, 1, 2, 3, 4 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @SuppressWarnings("deprecation")
  @Test
  public void testByteArrayFieldFixedSize() {
    testByteArrayFieldFixedSize(PrimitiveFieldType.INT8);
    testByteArrayFieldFixedSize(PrimitiveFieldType.BYTE);
    testByteArrayFieldFixedSize(PrimitiveFieldType.UINT8);
    testByteArrayFieldFixedSize(PrimitiveFieldType.CHAR);
  }

  private void testByteArrayFieldFixedSize(FieldType type) {
    ByteArrayField field = ByteArrayField.newVariable(type, "foo", 4);
    field.setValue(new byte[] { 1, 2, 3, 4 });
    assertEquals(type, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 1, 2, 3, 4 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testDoubleArrayFieldVariableSize() {
    DoubleArrayField field = DoubleArrayField.newVariable("foo", -1);
    field.setValue(new double[] { 1, 2, 3, 4 });
    assertEquals(PrimitiveFieldType.FLOAT64, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected =
        new byte[] { 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, 63, 0, 0, 0, 0, 0, 0, 0, 64, 0, 0, 0, 0, 0,
            0, 8, 64, 0, 0, 0, 0, 0, 0, 16, 64 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testDoubleArrayFieldFixedSize() {
    DoubleArrayField field = DoubleArrayField.newVariable("foo", 4);
    field.setValue(new double[] { 1, 2, 3, 4 });
    assertEquals(PrimitiveFieldType.FLOAT64, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected =
        new byte[] { 0, 0, 0, 0, 0, 0, -16, 63, 0, 0, 0, 0, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 8, 64,
            0, 0, 0, 0, 0, 0, 16, 64 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testFloatArrayFieldVariableSize() {
    FloatArrayField field = FloatArrayField.newVariable("foo", -1);
    field.setValue(new float[] { 1, 2, 3, 4 });
    assertEquals(PrimitiveFieldType.FLOAT32, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected =
        new byte[] { 4, 0, 0, 0, 0, 0, -128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, -128, 64 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testFloatArrayFieldFixedSize() {
    FloatArrayField field = FloatArrayField.newVariable("foo", 4);
    field.setValue(new float[] { 1, 2, 3, 4 });
    assertEquals(PrimitiveFieldType.FLOAT32, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 0, 0, -128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, -128, 64 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testIntegerArrayFieldVariableSize() {
    testIntegerArrayFieldVariableSize(PrimitiveFieldType.INT32);
    testIntegerArrayFieldVariableSize(PrimitiveFieldType.UINT32);
  }

  private void testIntegerArrayFieldVariableSize(FieldType type) {
    IntegerArrayField field = IntegerArrayField.newVariable(type, "foo", -1);
    field.setValue(new int[] { 1, 2, 3, 4 });
    assertEquals(type, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 4, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testIntegerArrayFieldFixedSize() {
    testIntegerArrayFieldFixedSize(PrimitiveFieldType.INT32);
    testIntegerArrayFieldFixedSize(PrimitiveFieldType.UINT32);
  }

  private void testIntegerArrayFieldFixedSize(FieldType type) {
    IntegerArrayField field = IntegerArrayField.newVariable(type, "foo", 4);
    field.setValue(new int[] { 1, 2, 3, 4 });
    assertEquals(type, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testLongArrayFieldVariableSize() {
    testLongArrayFieldVariableSize(PrimitiveFieldType.INT64);
    testLongArrayFieldVariableSize(PrimitiveFieldType.UINT64);
  }

  private void testLongArrayFieldVariableSize(FieldType type) {
    LongArrayField field = LongArrayField.newVariable(type, "foo", -1);
    field.setValue(new long[] { 1, 2, 3, 4 });
    assertEquals(type, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected =
        new byte[] { 4, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0,
            0, 0, 4, 0, 0, 0, 0, 0, 0, 0 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testLongArrayFieldFixedSize() {
    testLongArrayFieldFixedSize(PrimitiveFieldType.INT64);
    testLongArrayFieldFixedSize(PrimitiveFieldType.UINT64);
  }

  private void testLongArrayFieldFixedSize(FieldType type) {
    LongArrayField field = LongArrayField.newVariable(type, "foo", 4);
    field.setValue(new long[] { 1, 2, 3, 4 });
    assertEquals(type, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected =
        new byte[] { 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 4, 0,
            0, 0, 0, 0, 0, 0 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testShortArrayFieldVariableSize() {
    testShortArrayFieldVariableSize(PrimitiveFieldType.INT16);
    testShortArrayFieldVariableSize(PrimitiveFieldType.UINT16);
  }

  private void testShortArrayFieldVariableSize(FieldType type) {
    ShortArrayField field = ShortArrayField.newVariable(type, "foo", -1);
    field.setValue(new short[] { 1, 2, 3, 4 });
    assertEquals(type, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 4, 0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }

  @Test
  public void testShortArrayFieldFixedSize() {
    testShortArrayFieldFixedSize(PrimitiveFieldType.INT16);
    testShortArrayFieldFixedSize(PrimitiveFieldType.UINT16);
  }

  private void testShortArrayFieldFixedSize(FieldType type) {
    ShortArrayField field = ShortArrayField.newVariable(type, "foo", 4);
    field.setValue(new short[] { 1, 2, 3, 4 });
    assertEquals(type, field.getType());
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    field.serialize(buffer);
    byte[] expected = new byte[] { 1, 0, 2, 0, 3, 0, 4, 0 };
    byte[] actual = new byte[buffer.readableBytes()];
    buffer.readBytes(actual);
    assertArrayEquals(expected, actual);
  }
}
