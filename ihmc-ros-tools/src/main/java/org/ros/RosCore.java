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

package org.ros;

import com.google.common.annotations.VisibleForTesting;

import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.internal.node.server.master.MasterServer;

import java.net.URI;
import java.util.concurrent.TimeUnit;

// TODO(damonkohler): Add /rosout node.
/**
 * {@link RosCore} is a collection of nodes and programs that are pre-requisites
 * of a ROS-based system. You must have a {@link RosCore} (either this
 * implementation or the default Python implementation distributed with ROS)
 * running in order for ROS nodes to communicate.
 * 
 * @see <a href="http://www.ros.org/wiki/roscore">roscore documentation</a>
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class RosCore {

  private final MasterServer masterServer;

  public static RosCore newPublic(String host, int port) {
    return new RosCore(BindAddress.newPublic(port), new AdvertiseAddress(host));
  }

  public static RosCore newPublic(int port) {
    return new RosCore(BindAddress.newPublic(port), AdvertiseAddress.newPublic());
  }

  public static RosCore newPublic() {
    return new RosCore(BindAddress.newPublic(), AdvertiseAddress.newPublic());
  }

  public static RosCore newPrivate(int port) {
    return new RosCore(BindAddress.newPrivate(port), AdvertiseAddress.newPrivate());
  }

  public static RosCore newPrivate() {
    return new RosCore(BindAddress.newPrivate(), AdvertiseAddress.newPrivate());
  }

  private RosCore(BindAddress bindAddress, AdvertiseAddress advertiseAddress) {
    masterServer = new MasterServer(bindAddress, advertiseAddress);
  }

  public void start() {
    masterServer.start();
  }

  public URI getUri() {
    return masterServer.getUri();
  }

  public void awaitStart() throws InterruptedException {
    masterServer.awaitStart();
  }

  public boolean awaitStart(long timeout, TimeUnit unit) throws InterruptedException {
    return masterServer.awaitStart(timeout, unit);
  }

  public void shutdown() {
    masterServer.shutdown();
  }

  @VisibleForTesting
  public MasterServer getMasterServer() {
    return masterServer;
  }
}
