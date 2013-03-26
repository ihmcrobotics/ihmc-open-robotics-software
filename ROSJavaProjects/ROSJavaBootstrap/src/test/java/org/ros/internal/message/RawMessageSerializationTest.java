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

import static org.junit.Assert.assertTrue;

import com.google.common.collect.Lists;

import org.jboss.netty.buffer.ChannelBuffer;
import org.junit.Before;
import org.junit.Test;
import org.ros.internal.message.topic.TopicDefinitionResourceProvider;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class RawMessageSerializationTest {

  private TopicDefinitionResourceProvider topicDefinitionResourceProvider;
  private MessageFactory messageFactory;

  @Before
  public void before() {
    topicDefinitionResourceProvider = new TopicDefinitionResourceProvider();
    messageFactory = new DefaultMessageFactory(topicDefinitionResourceProvider);
  }

  private void checkSerializeAndDeserialize(Message message) {
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    DefaultMessageSerializer serializer = new DefaultMessageSerializer();
    serializer.serialize(message, buffer);
    DefaultMessageDeserializer<RawMessage> deserializer =
        new DefaultMessageDeserializer<RawMessage>(message.toRawMessage().getIdentifier(),
            messageFactory);
    RawMessage deserializedMessage = deserializer.deserialize(buffer);
    assertTrue(message.equals(deserializedMessage));
  }

  @Test
  public void testBool() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Bool");
    rawMessage.setBool("data", true);
    checkSerializeAndDeserialize(rawMessage);
    rawMessage.setBool("data", false);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testInt8() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Int8");
    rawMessage.setInt8("data", (byte) 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testUint8() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/UInt8");
    rawMessage.setUInt8("data", (byte) 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testInt16() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Int16");
    rawMessage.setInt16("data", (short) 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testUInt16() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/UInt16");
    rawMessage.setUInt16("data", (short) 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testInt32() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Int32");
    rawMessage.setInt32("data", 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testUInt32() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/UInt32");
    rawMessage.setUInt32("data", 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testInt64() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Int64");
    rawMessage.setInt64("data", 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testUInt64() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/UInt64");
    rawMessage.setUInt64("data", 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testFloat32() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Float32");
    rawMessage.setFloat32("data", 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testFloat64() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Float64");
    rawMessage.setFloat64("data", 42);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testString() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/String");
    rawMessage.setString("data", "Hello, ROS!");
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testTime() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Time");
    rawMessage.setTime("data", new Time());
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testDuration() {
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Duration");
    rawMessage.setDuration("data", new Duration());
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testNestedMessage() {
    topicDefinitionResourceProvider.add("foo/foo", "std_msgs/String data");
    RawMessage fooMessage = messageFactory.newFromType("foo/foo");
    RawMessage stringMessage = messageFactory.newFromType("std_msgs/String");
    stringMessage.setString("data", "Hello, ROS!");
    fooMessage.setMessage("data", stringMessage);
    checkSerializeAndDeserialize(fooMessage);
  }

  @Test
  public void testNestedMessageArray() {
    topicDefinitionResourceProvider.add("foo/foo", "std_msgs/String[] data");
    RawMessage fooMessage = messageFactory.newFromType("foo/foo");
    RawMessage stringMessageA = messageFactory.newFromType("std_msgs/String");
    stringMessageA.setString("data", "Hello, ROS!");
    RawMessage stringMessageB = messageFactory.newFromType("std_msgs/String");
    stringMessageB.setString("data", "Goodbye, ROS!");
    fooMessage.setMessageList("data", Lists.<Message>newArrayList(stringMessageA, stringMessageB));
    checkSerializeAndDeserialize(fooMessage);
  }

  @Test
  public void testChannelBuffer() {
    topicDefinitionResourceProvider.add("foo/foo", "uint8[] data");
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    buffer.writeBytes(new byte[] { 1, 2, 3, 4, 5 });
    RawMessage rawMessage = messageFactory.newFromType("foo/foo");
    rawMessage.setChannelBuffer("data", buffer);
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testInt32Array() {
    topicDefinitionResourceProvider.add("foo/foo", "int32[] data");
    RawMessage rawMessage = messageFactory.newFromType("foo/foo");
    rawMessage.setInt32Array("data", new int[] { 1, 2, 3, 4, 5 });
    checkSerializeAndDeserialize(rawMessage);
  }

  @Test
  public void testFloat64Array() {
    topicDefinitionResourceProvider.add("foo/foo", "float64[] data");
    RawMessage rawMessage = messageFactory.newFromType("foo/foo");
    rawMessage.setFloat64Array("data", new double[] { 1, 2, 3, 4, 5 });
    checkSerializeAndDeserialize(rawMessage);
  }
}
