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

package org.ros.internal.node.xmlrpc;

import static org.junit.Assert.assertEquals;
import static org.mockito.Matchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.google.common.collect.Lists;

import org.junit.Test;
import org.mockito.Matchers;
import org.ros.internal.node.response.StatusCode;
import org.ros.internal.node.server.master.MasterServer;
import org.ros.namespace.GraphName;

import java.net.URI;
import java.util.List;

/**
 * Tests for the {@link MasterXmlRpcEndpointImpl}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MasterXmlRpcEndpointImplTest {

  @Test
  public void testGetUri() throws Exception {
    URI testUri = new URI("http://foo.bar:8080");
    MasterServer mockMaster = mock(MasterServer.class);
    when(mockMaster.getUri()).thenReturn(testUri);
    MasterXmlRpcEndpointImpl master = new MasterXmlRpcEndpointImpl(mockMaster);
    List<Object> response = master.getUri("/caller");
    assertEquals(StatusCode.SUCCESS.toInt(), response.get(0));
    assertEquals(testUri.toString(), response.get(2));
  }

  @Test
  public void testLookupNodeExisting() throws Exception {
    MasterServer mockMaster = mock(MasterServer.class);
    final GraphName nodeName = GraphName.of("/foo");
    final URI nodeSlaveUri = new URI("http://bar");
    when(mockMaster.lookupNode(eq(nodeName))).thenReturn(nodeSlaveUri);
    MasterXmlRpcEndpointImpl master = new MasterXmlRpcEndpointImpl(mockMaster);
    List<Object> response = master.lookupNode("/caller", nodeName.toString());
    assertEquals(StatusCode.SUCCESS.toInt(), response.get(0));
    assertEquals(nodeSlaveUri.toString(), response.get(2));
  }

  @Test
  public void testLookupNodeNotExisting() throws Exception {
    MasterServer mockMaster = mock(MasterServer.class);
    when(mockMaster.lookupNode(Matchers.<GraphName>any())).thenReturn(null);
    MasterXmlRpcEndpointImpl master = new MasterXmlRpcEndpointImpl(mockMaster);
    List<Object> response = master.lookupNode("/caller", "/foo");
    assertEquals(StatusCode.ERROR.toInt(), response.get(0));
    assertEquals("null", response.get(2));
  }

  @Test
  public void testRegisterPublisherWithNoSubscribers() {
    MasterServer mockMaster = mock(MasterServer.class);
    when(
        mockMaster.registerPublisher(Matchers.<GraphName>any(), Matchers.<URI>any(),
            Matchers.<GraphName>any(), Matchers.<String>any())).thenReturn(
        Lists.<URI>newArrayList());
    MasterXmlRpcEndpointImpl master = new MasterXmlRpcEndpointImpl(mockMaster);
    List<Object> response = master.registerPublisher("/caller", "/foo", "/bar", "http://baz");
    assertEquals(StatusCode.SUCCESS.toInt(), response.get(0));
    assertEquals(Lists.newArrayList(), response.get(2));
  }

  @Test
  public void testRegisterPublisher() throws Exception {
    MasterServer mockMaster = mock(MasterServer.class);
    final GraphName nodeName = GraphName.of("/slave");
    final URI nodeSlaveUri = new URI("http://api");
    final GraphName topicName = GraphName.of("/topic");
    final String messageType = "/topicType";
    when(
        mockMaster
            .registerPublisher(eq(nodeName), eq(nodeSlaveUri), eq(topicName), eq(messageType)))
        .thenReturn(Lists.<URI>newArrayList(nodeSlaveUri));
    MasterXmlRpcEndpointImpl master = new MasterXmlRpcEndpointImpl(mockMaster);
    List<Object> response =
        master.registerPublisher(nodeName.toString(), topicName.toString(), messageType,
            nodeSlaveUri.toString());
    assertEquals(StatusCode.SUCCESS.toInt(), response.get(0));
    assertEquals(Lists.newArrayList(nodeSlaveUri.toString()), response.get(2));
  }

  @Test
  public void testRegisterSubscriberWithNoSubscribers() {
    MasterServer mockMaster = mock(MasterServer.class);
    when(
        mockMaster.registerSubscriber(Matchers.<GraphName>any(), Matchers.<URI>any(),
            Matchers.<GraphName>any(), Matchers.<String>any())).thenReturn(
        Lists.<URI>newArrayList());
    MasterXmlRpcEndpointImpl master = new MasterXmlRpcEndpointImpl(mockMaster);
    List<Object> response = master.registerSubscriber("/caller", "/foo", "/bar", "http://baz");
    assertEquals(StatusCode.SUCCESS.toInt(), response.get(0));
    assertEquals(Lists.newArrayList(), response.get(2));
  }

  @Test
  public void testRegisterSubscriber() throws Exception {
    MasterServer mockMaster = mock(MasterServer.class);
    final GraphName nodeName = GraphName.of("/slave");
    final URI nodeSlaveUri = new URI("http://api");
    final GraphName topicName = GraphName.of("/topic");
    final String topicMessageType = "/topicType";

    when(
        mockMaster.registerSubscriber(eq(nodeName), eq(nodeSlaveUri), eq(topicName),
            eq(topicMessageType))).thenReturn(Lists.<URI>newArrayList(nodeSlaveUri));
    MasterXmlRpcEndpointImpl master = new MasterXmlRpcEndpointImpl(mockMaster);
    List<Object> response =
        master.registerSubscriber(nodeName.toString(), topicName.toString(), topicMessageType,
            nodeSlaveUri.toString());
    assertEquals(StatusCode.SUCCESS.toInt(), response.get(0));
    assertEquals(Lists.newArrayList(nodeSlaveUri.toString()), response.get(2));
  }
}
