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

package org.ros.internal.node.topic;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.exception.RemoteException;
import org.ros.internal.node.client.SlaveClient;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.server.SlaveServer;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.internal.transport.ProtocolDescription;
import org.ros.internal.transport.ProtocolNames;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * A {@link Runnable} which is used whenever new publishers are being added to a
 * {@link DefaultSubscriber}. It takes care of registration between the {@link Subscriber}
 * and remote {@link Publisher}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
class UpdatePublisherRunnable<MessageType> implements Runnable {

  private static final Log log = LogFactory.getLog(UpdatePublisherRunnable.class);

  private final DefaultSubscriber<MessageType> subscriber;
  private final PublisherIdentifier publisherIdentifier;
  private final NodeIdentifier nodeIdentifier;

  /**
   * @param subscriber
   * @param nodeIdentifier
   *          {@link NodeIdentifier} of the {@link Subscriber}'s
   *          {@link SlaveServer}
   * @param publisherIdentifier
   *          {@link PublisherIdentifier} of the new {@link Publisher}
   */
  public UpdatePublisherRunnable(DefaultSubscriber<MessageType> subscriber,
      NodeIdentifier nodeIdentifier, PublisherIdentifier publisherIdentifier) {
    this.subscriber = subscriber;
    this.nodeIdentifier = nodeIdentifier;
    this.publisherIdentifier = publisherIdentifier;
  }

  @Override
  public void run() {
    SlaveClient slaveClient;
    try {
      slaveClient = new SlaveClient(nodeIdentifier.getName(), publisherIdentifier.getNodeUri());
      Response<ProtocolDescription> response =
          slaveClient.requestTopic(subscriber.getTopicName(), ProtocolNames.SUPPORTED);
      // TODO(kwc): all of this logic really belongs in a protocol handler
      // registry.
      ProtocolDescription selected = response.getResult();
      if (ProtocolNames.SUPPORTED.contains(selected.getName())) {
        subscriber.addPublisher(publisherIdentifier, selected.getAddress());
      } else {
        log.error("Publisher returned unsupported protocol selection: " + response);
      }
    } catch (RemoteException e) {
      // TODO(damonkohler): Retry logic is needed at the XML-RPC layer.
      log.error(e);
    } catch (XmlRpcTimeoutException e) {
      // TODO(damonkohler): see above note re: retry
      log.error(e);
    } catch (RuntimeException e) {
      // TODO(kwc):
      // org.apache.xmlrpc.XmlRpcException/java.net.ConnectException's are
      // leaking through as java.lang.reflect.UndeclaredThrowableExceptions.
      // This is happening whenever the node attempts to connect to a stale
      // publisher (i.e. a publisher that is no longer online).
      log.error(e);
    }
  }
}
