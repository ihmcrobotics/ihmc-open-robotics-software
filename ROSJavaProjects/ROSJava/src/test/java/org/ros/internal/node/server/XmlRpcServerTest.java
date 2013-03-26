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

package org.ros.internal.node.server;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.net.URI;

import org.junit.Test;
import org.ros.address.Address;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.internal.node.xmlrpc.XmlRpcEndpoint;

/**
 * @author kwc@willowgarage.com (Ken Conley)
 */
public class XmlRpcServerTest {

  class FakeNode implements XmlRpcEndpoint {
  }

  @Test
  public void testGetPublicUri() {
    BindAddress bindAddress = BindAddress.newPublic();
    XmlRpcServer xmlRpcServer = new XmlRpcServer(bindAddress, new AdvertiseAddress("override"));
    try {
      xmlRpcServer.getUri();
      fail("Should not have succeeded before startup.");
    } catch (RuntimeException e) {
    }

    xmlRpcServer.start(FakeNode.class, new FakeNode());
    URI uri = xmlRpcServer.getUri();
    assertEquals("override", uri.getHost());
    assertTrue(uri.getPort() > 0);

    xmlRpcServer.shutdown();
  }
  
  @Test
  public void testGetPrivateUri() {
    BindAddress bindAddress = BindAddress.newPrivate();
    XmlRpcServer xmlRpcServer = new XmlRpcServer(bindAddress, AdvertiseAddress.newPrivate());
    try {
      xmlRpcServer.getUri();
      fail("Should not have succeeded before startup.");
    } catch (RuntimeException e) {
    }

    xmlRpcServer.start(FakeNode.class, new FakeNode());
    URI uri = xmlRpcServer.getUri();
    assertEquals(Address.LOOPBACK, uri.getHost());
    assertTrue(uri.getPort() > 0);

    xmlRpcServer.shutdown();
  }
}
