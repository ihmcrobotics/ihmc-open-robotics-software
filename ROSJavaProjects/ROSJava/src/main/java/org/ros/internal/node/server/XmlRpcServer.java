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

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.server.PropertyHandlerMapping;
import org.apache.xmlrpc.server.XmlRpcServerConfigImpl;
import org.apache.xmlrpc.webserver.WebServer;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.system.Process;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URI;
import java.util.concurrent.Callable;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * Base class for an XML-RPC server.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class XmlRpcServer {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(XmlRpcServer.class);

  private final WebServer server;
  private final AdvertiseAddress advertiseAddress;
  private final CountDownLatch startLatch;

  public XmlRpcServer(BindAddress bindAddress, AdvertiseAddress advertiseAddress) {
    InetSocketAddress address = bindAddress.toInetSocketAddress();
    server = new WebServer(address.getPort(), address.getAddress());
    this.advertiseAddress = advertiseAddress;
    this.advertiseAddress.setPortCallable(new Callable<Integer>() {
      @Override
      public Integer call() throws Exception {
        return server.getPort();
      }
    });
    startLatch = new CountDownLatch(1);
  }

  /**
   * Start up the remote calling server.
   * 
   * @param instanceClass
   *          the class of the remoting server
   * 
   * @param instance
   *          an instance of the remoting server class
   */
  public <T extends org.ros.internal.node.xmlrpc.XmlRpcEndpoint> void start(Class<T> instanceClass,
      T instance) {
    org.apache.xmlrpc.server.XmlRpcServer xmlRpcServer = server.getXmlRpcServer();
    PropertyHandlerMapping phm = new PropertyHandlerMapping();
    phm.setRequestProcessorFactoryFactory(new NodeRequestProcessorFactoryFactory<T>(instance));
    try {
      phm.addHandler("", instanceClass);
    } catch (XmlRpcException e) {
      throw new RosRuntimeException(e);
    }
    xmlRpcServer.setHandlerMapping(phm);
    XmlRpcServerConfigImpl serverConfig = (XmlRpcServerConfigImpl) xmlRpcServer.getConfig();
    serverConfig.setEnabledForExtensions(false);
    serverConfig.setContentLengthOptional(false);
    try {
      server.start();
    } catch (IOException e) {
      throw new RosRuntimeException(e);
    }
    if (DEBUG) {
      log.info("Bound to: " + getUri());
    }
    startLatch.countDown();
  }

  /**
   * Shut the remote call server down.
   */
  public void shutdown() {
    server.shutdown();
  }

  /**
   * @return the {@link URI} of the server
   */
  public URI getUri() {
    return advertiseAddress.toUri("http");
  }

  public InetSocketAddress getAddress() {
    return advertiseAddress.toInetSocketAddress();
  }

  public AdvertiseAddress getAdvertiseAddress() {
    return advertiseAddress;
  }

  public void awaitStart() throws InterruptedException {
    startLatch.await();
  }

  public boolean awaitStart(long timeout, TimeUnit unit) throws InterruptedException {
    return startLatch.await(timeout, unit);
  }

  /**
   * @return PID of node process if available, throws
   *         {@link UnsupportedOperationException} otherwise.
   */
  public int getPid() {
    return Process.getPid();
  }
}
