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

package org.ros.address;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.net.InetAddress;
import java.net.UnknownHostException;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class AddressTest {

  @Test
  public void testLocalhost() {
    InetAddress localhost = InetAddressFactory.newFromHostString(Address.LOCALHOST);
    assertEquals(Address.LOCALHOST, localhost.getHostName());
    assertTrue(localhost.isLoopbackAddress());
  }

  @Test
  public void testLoopback() {
    InetAddress loopback = InetAddressFactory.newFromHostString(Address.LOOPBACK);
    assertEquals(Address.LOOPBACK, loopback.getHostName());
    assertTrue(loopback.isLoopbackAddress());
  }

  // TODO(damonkohler): This test fails if the host isn't registered with DNS.
  // These tests need to be changed to inject behavior into the
  // getAllInetAddressesByName() method.
  public void testPublicHost() throws UnknownHostException {
    String host = InetAddress.getLocalHost().getCanonicalHostName();
    InetAddress publicHost = InetAddressFactory.newFromHostString(host);
    assertEquals(host, publicHost.getHostName());
    assertFalse(publicHost.isLoopbackAddress());
  }

  @Test
  public void testPublicIpv4() {
    String host = "1.2.3.4";
    InetAddress publicHost = InetAddressFactory.newFromHostString(host);
    assertEquals(host, publicHost.getHostName());
    assertFalse(publicHost.isLoopbackAddress());
  }
}
