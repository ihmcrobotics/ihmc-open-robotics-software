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

package org.ros.internal.transport.tcp;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.address.InetAddressFactory;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

/**
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TcpRosServerTest {
  private ScheduledExecutorService executorService;

  @Before
  public void setup() {
    executorService = Executors.newScheduledThreadPool(10);
  }

  @After
  public void tearDown() {
    executorService.shutdown();
  }

  @Test
  public void testGetAddressFailsIfServerNotRunning() throws UnknownHostException {
    TcpRosServer tcpRosServer =
        new TcpRosServer(BindAddress.newPublic(), AdvertiseAddress.newPublic(), null, null,
            executorService);

    try {
      tcpRosServer.getAddress();
      fail();
    } catch (RuntimeException e) {
      // getAddress() must fail when the server is not running.
    }

    tcpRosServer.start();
    InetSocketAddress address = tcpRosServer.getAddress();
    assertTrue(address.getPort() > 0);
    assertEquals(InetAddressFactory.newNonLoopback().getCanonicalHostName(), address.getAddress()
        .getHostName());
    tcpRosServer.shutdown();

    try {
      tcpRosServer.getAddress();
      fail();
    } catch (RuntimeException e) {
      // getAddress() must fail when the server is not running.
    }
  }

  @Test
  public void testFailIfPortTaken() {
    TcpRosServer firstServer =
        new TcpRosServer(BindAddress.newPublic(), AdvertiseAddress.newPublic(), null, null,
            executorService);
    firstServer.start();
    try {
      TcpRosServer secondServer =
          new TcpRosServer(BindAddress.newPublic(firstServer.getAddress().getPort()),
              AdvertiseAddress.newPublic(), null, null, executorService);
      secondServer.start();
      fail();
    } catch (RuntimeException e) {
      // Starting a server on an already used port must fail.
    }
    firstServer.shutdown();
  }

  @Test
  public void testFailIfStartedWhileRunning() {
    TcpRosServer tcpRosServer =
        new TcpRosServer(BindAddress.newPublic(), AdvertiseAddress.newPublic(), null, null,
            executorService);
    tcpRosServer.start();
    try {
      tcpRosServer.start();
      fail();
    } catch (RuntimeException e) {
      // Starting the server twice must fail.
    }
    tcpRosServer.shutdown();
  }
}
