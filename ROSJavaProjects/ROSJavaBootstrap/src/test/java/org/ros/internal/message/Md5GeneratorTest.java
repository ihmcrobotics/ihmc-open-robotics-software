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

import org.ros.internal.message.definition.MessageDefinitionProviderChain;

import org.junit.Before;
import org.junit.Test;
import org.ros.internal.message.service.ServiceDefinitionResourceProvider;
import org.ros.internal.message.service.ServiceDescription;
import org.ros.internal.message.service.ServiceDescriptionFactory;
import org.ros.internal.message.topic.TopicDefinitionResourceProvider;
import org.ros.internal.message.topic.TopicDescription;
import org.ros.internal.message.topic.TopicDescriptionFactory;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class Md5GeneratorTest {

  private TopicDescriptionFactory topicDescriptionFactory;
  private ServiceDescriptionFactory serviceDescriptionFactory;

  @Before
  public void setUp() {
    MessageDefinitionProviderChain messageDefinitionProviderChain =
        new MessageDefinitionProviderChain();
    messageDefinitionProviderChain
        .addMessageDefinitionProvider(new TopicDefinitionResourceProvider());
    messageDefinitionProviderChain
        .addMessageDefinitionProvider(new ServiceDefinitionResourceProvider());
    topicDescriptionFactory = new TopicDescriptionFactory(messageDefinitionProviderChain);
    serviceDescriptionFactory = new ServiceDescriptionFactory(messageDefinitionProviderChain);
  }

  @Test
  public void testPrimitives() {
    TopicDescription topicDescription =
        topicDescriptionFactory.newFromType("test_ros/TestPrimitives");
    assertEquals("3e70f428a22c0d26ca67f87802c8e00f", topicDescription.getMd5Checksum());
  }

  @Test
  public void testString() {
    TopicDescription topicDescription = topicDescriptionFactory.newFromType("test_ros/TestString");
    assertEquals("334ff4377be93faa44ebc66d23d40fd3", topicDescription.getMd5Checksum());
  }

  @Test
  public void testHeader() {
    TopicDescription topicDescription = topicDescriptionFactory.newFromType("test_ros/TestHeader");
    assertEquals("4b5a00f536da2f756ba6aebcf795a967", topicDescription.getMd5Checksum());
  }

  @Test
  public void testArrays() {
    TopicDescription topicDescription = topicDescriptionFactory.newFromType("test_ros/TestArrays");
    assertEquals("4cc9b5e2cebe791aa3e994f5bc159eb6", topicDescription.getMd5Checksum());
  }

  @Test
  public void testComposite() {
    TopicDescription topicDescription = topicDescriptionFactory.newFromType("test_ros/Composite");
    assertEquals("d8fb6eb869ad3956b50e8737d96dc9fa", topicDescription.getMd5Checksum());
  }

  @Test
  public void testOdometry() {
    TopicDescription topicDescription = topicDescriptionFactory.newFromType("nav_msgs/Odometry");
    assertEquals("cd5e73d190d741a2f92e81eda573aca7", topicDescription.getMd5Checksum());
  }

  @Test
  public void testEmpty() {
    ServiceDescription serviceDescription = serviceDescriptionFactory.newFromType("std_srvs/Empty");
    assertEquals("d41d8cd98f00b204e9800998ecf8427e", serviceDescription.getMd5Checksum());
  }

  @Test
  public void testAddTwoInts() {
    ServiceDescription serviceDescription =
        serviceDescriptionFactory.newFromType("test_ros/AddTwoInts");
    assertEquals("6a2e34150c00229791cc89ff309fff21", serviceDescription.getMd5Checksum());
  }

  @Test
  public void testTransitiveSrv() {
    ServiceDescription serviceDescription =
        serviceDescriptionFactory.newFromType("test_rospy/TransitiveSrv");
    assertEquals("8b7918ee2b81eaf825f4c70de011f6fa", serviceDescription.getMd5Checksum());
  }
}
