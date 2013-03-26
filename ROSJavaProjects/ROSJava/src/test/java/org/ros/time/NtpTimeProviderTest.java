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

package org.ros.time;

import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.ros.RosTest;
import org.ros.address.InetAddressFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import java.io.IOException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class NtpTimeProviderTest extends RosTest {

  @Test
  public void testNtpUbuntuCom() throws InterruptedException {
    final NtpTimeProvider ntpTimeProvider =
        new NtpTimeProvider(InetAddressFactory.newFromHostString("ntp.ubuntu.com"),
            Executors.newScheduledThreadPool(Integer.MAX_VALUE));
    final CountDownLatch latch = new CountDownLatch(1);
    nodeConfiguration.setTimeProvider(ntpTimeProvider);
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("node");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        try {
          ntpTimeProvider.updateTime();
        } catch (IOException e) {
          // Ignored. This is only a sanity check.
        }
        ntpTimeProvider.getCurrentTime();
        System.out.println("System time: " + System.currentTimeMillis());
        System.out.println("NTP time: " + ntpTimeProvider.getCurrentTime().totalNsecs() / 1000000);
        latch.countDown();
      }
    }, nodeConfiguration);
    assertTrue(latch.await(1, TimeUnit.SECONDS));
  }
}
