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

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;
import org.ros.internal.message.topic.TopicDefinitionResourceProvider;
import org.ros.message.MessageFactory;

import com.google.common.collect.Lists;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageTest {

  private TopicDefinitionResourceProvider topicDefinitionResourceProvider;
  private MessageFactory messageFactory;

  @Before
  public void before() {
    topicDefinitionResourceProvider = new TopicDefinitionResourceProvider();
    messageFactory = new DefaultMessageFactory(topicDefinitionResourceProvider);
  }

  @Test(timeout=300000)
  public void testCreateEmptyMessage() {
    topicDefinitionResourceProvider.add("foo/foo", "");
    messageFactory.newFromType("foo/foo");
  }

  @Test(timeout=300000)
  public void testCreateEmptyMessageWithBlankLines() {
    topicDefinitionResourceProvider.add("foo/foo", "\n\n\n\n\n");
    messageFactory.newFromType("foo/foo");
  }

  @Test(timeout=300000)
  public void testString() {
    String data = "Hello, ROS!";
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/String");
    rawMessage.setString("data", data);
    assertEquals(data, rawMessage.getString("data"));
  }

  @Test(timeout=300000)
  public void testStringWithComments() {
    topicDefinitionResourceProvider.add("foo/foo", "# foo\nstring data\n    # string other data");
    String data = "Hello, ROS!";
    RawMessage rawMessage = messageFactory.newFromType("foo/foo");
    rawMessage.setString("data", data);
    assertEquals(data, rawMessage.getString("data"));
  }

  @Test(timeout=300000)
  public void testInt8() {
    byte data = 42;
    RawMessage rawMessage = messageFactory.newFromType("std_msgs/Int8");
    rawMessage.setInt8("data", data);
    assertEquals(data, rawMessage.getInt8("data"));
  }

  @Test(timeout=300000)
  public void testNestedMessage() {
    topicDefinitionResourceProvider.add("foo/foo", "bar data");
    topicDefinitionResourceProvider.add("foo/bar", "int8 data");
    RawMessage fooMessage = messageFactory.newFromType("foo/foo");
    RawMessage barMessage = messageFactory.newFromType("foo/bar");
    fooMessage.setMessage("data", barMessage);
    byte data = 42;
    barMessage.setInt8("data", data);
    assertEquals(data, fooMessage.getMessage("data").toRawMessage().getInt8("data"));
  }

  @Test(timeout=300000)
  public void testNestedMessageList() {
    topicDefinitionResourceProvider.add("foo/foo", "bar[] data");
    topicDefinitionResourceProvider.add("foo/bar", "int8 data");
    RawMessage fooMessage = messageFactory.newFromType("foo/foo");
    RawMessage barMessage = messageFactory.newFromType("foo/bar");
    fooMessage.setMessageList("data", Lists.<Message>newArrayList(barMessage));
    byte data = 42;
    barMessage.toRawMessage().setInt8("data", data);
    assertEquals(data, fooMessage.getMessageList("data").get(0).toRawMessage().getInt8("data"));
  }

  @Test(timeout=300000)
  public void testConstantInt8() {
    topicDefinitionResourceProvider.add("foo/foo", "int8 data=42");
    RawMessage rawMessage = messageFactory.newFromType("foo/foo");
    assertEquals(42, rawMessage.getInt8("data"));
  }

  @Test(timeout=300000)
  public void testConstantString() {
    topicDefinitionResourceProvider.add("foo/foo", "string data=Hello, ROS! # comment ");
    RawMessage rawMessage = messageFactory.newFromType("foo/foo");
    assertEquals("Hello, ROS! # comment", rawMessage.getString("data"));
  }

  public void testInt8List() {
    topicDefinitionResourceProvider.add("foo/foo", "int8[] data");
    RawMessage rawMessage = messageFactory.newFromType("foo/foo");
    byte[] data = new byte[] { (byte) 1, (byte) 2, (byte) 3 };
    rawMessage.setInt8Array("data", data);
    assertEquals(data, rawMessage.getInt8Array("data"));
  }
}