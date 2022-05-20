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

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.StatusCode;
import org.ros.internal.node.server.ServerException;
import org.ros.internal.node.server.SlaveServer;
import org.ros.internal.node.topic.DefaultPublisher;
import org.ros.internal.node.topic.DefaultSubscriber;
import org.ros.internal.transport.ProtocolDescription;
import org.ros.namespace.GraphName;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class SlaveXmlRpcEndpointImpl implements SlaveXmlRpcEndpoint {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(SlaveXmlRpcEndpointImpl.class);

  private final SlaveServer slave;

  public SlaveXmlRpcEndpointImpl(SlaveServer slave) {
    this.slave = slave;
  }

  @Override
  public List<Object> getBusStats(String callerId) {
    return slave.getBusStats(callerId);
  }

  @Override
  public List<Object> getBusInfo(String callerId) {
    List<Object> busInfo = slave.getBusInfo(callerId);
    return Response.newSuccess("bus info", busInfo).toList();
  }

  @Override
  public List<Object> getMasterUri(String callerId) {
    URI uri = slave.getMasterUri();
    return new Response<String>(StatusCode.SUCCESS, "", uri.toString()).toList();
  }

  @Override
  public List<Object> shutdown(String callerId, String message) {
    log.info("Shutdown requested by " + callerId + " with message \"" + message + "\"");
    slave.shutdown();
    return Response.newSuccess("Shutdown successful.", null).toList();
  }

  @Override
  public List<Object> getPid(String callerId) {
    try {
      int pid = slave.getPid();
      return Response.newSuccess("PID: " + pid, pid).toList();
    } catch (UnsupportedOperationException e) {
      return Response.newFailure("Cannot retrieve PID on this platform.", -1).toList();
    }
  }

  @Override
  public List<Object> getSubscriptions(String callerId) {
    Collection<DefaultSubscriber<?>> subscribers = slave.getSubscriptions();
    List<List<String>> subscriptions = Lists.newArrayList();
    for (DefaultSubscriber<?> subscriber : subscribers) {
      subscriptions.add(subscriber.getTopicDeclarationAsList());
    }
    return Response.newSuccess("Success", subscriptions).toList();
  }

  @Override
  public List<Object> getPublications(String callerId) {
    Collection<DefaultPublisher<?>> publishers = slave.getPublications();
    List<List<String>> publications = Lists.newArrayList();
    for (DefaultPublisher<?> publisher : publishers) {
      publications.add(publisher.getTopicDeclarationAsList());
    }
    return Response.newSuccess("Success", publications).toList();
  }

  private List<Object> parameterUpdate(String parameterName, Object parameterValue) {
    if (slave.paramUpdate(GraphName.of(parameterName), parameterValue) > 0) {
      return Response.newSuccess("Success", null).toList();
    }
    return Response
        .newError("No subscribers for parameter key \"" + parameterName + "\".", null).toList();
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, boolean value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, char value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, byte value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, short value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, int value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, double value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, String value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, List<?> value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, Vector<?> value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> paramUpdate(String callerId, String key, Map<?, ?> value) {
    return parameterUpdate(key, value);
  }

  @Override
  public List<Object> publisherUpdate(String callerId, String topicName, Object[] publishers) {
    try {
      ArrayList<URI> publisherUris = new ArrayList<URI>(publishers.length);
      for (Object publisher : publishers) {
        URI uri = new URI(publisher.toString());
        if (!uri.getScheme().equals("http") && !uri.getScheme().equals("https")) {
          return Response.newError("Unknown URI scheme sent in update.", 0).toList();
        }
        publisherUris.add(uri);
      }
      slave.publisherUpdate(callerId, topicName, publisherUris);
      return Response.newSuccess("Publisher update received.", 0).toList();
    } catch (URISyntaxException e) {
      return Response.newError("Invalid URI sent in update.", 0).toList();
    }
  }

  @Override
  public List<Object> requestTopic(String callerId, String topic, Object[] protocols) {
    Set<String> requestedProtocols = Sets.newHashSet();
    for (int i = 0; i < protocols.length; i++) {
      requestedProtocols.add((String) ((Object[]) protocols[i])[0]);
    }
    ProtocolDescription protocol;
    try {
      protocol = slave.requestTopic(topic, requestedProtocols);
    } catch (ServerException e) {
      return Response.newError(e.getMessage(), null).toList();
    }
    List<Object> response = Response.newSuccess(protocol.toString(), protocol.toList()).toList();
    if (DEBUG) {
      log.info("requestTopic(" + topic + ", " + requestedProtocols + ") response: "
          + response.toString());
    }
    return response;
  }
}
