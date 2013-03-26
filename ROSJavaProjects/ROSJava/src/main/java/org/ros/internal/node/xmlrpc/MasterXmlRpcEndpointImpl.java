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

import org.ros.exception.RosRuntimeException;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.server.ParameterServer;
import org.ros.internal.node.server.master.MasterServer;
import org.ros.namespace.GraphName;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A combined XML-RPC endpoint for the master and parameter servers.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MasterXmlRpcEndpointImpl implements MasterXmlRpcEndpoint,
    ParameterServerXmlRpcEndpoint {

  private final MasterServer master;
  private final ParameterServer parameterServer;

  public MasterXmlRpcEndpointImpl(MasterServer master) {
    this.master = master;
    parameterServer = new ParameterServer();
  }

  @Override
  public List<Object> getPid(String callerId) {
    return Response.newSuccess("server pid", master.getPid()).toList();
  }

  @Override
  public List<Object> getPublishedTopics(String callerId, String subgraph) {
    return Response.newSuccess("current topics",
        master.getPublishedTopics(GraphName.of(callerId), GraphName.of(subgraph))).toList();
  }

  @Override
  public List<Object> getTopicTypes(String callerId) {
    return Response.newSuccess("topic types", master.getTopicTypes(GraphName.of(callerId)))
        .toList();
  }

  @Override
  public List<Object> getSystemState(String callerId) {
    return Response.newSuccess("current system state", master.getSystemState()).toList();
  }

  @Override
  public List<Object> getUri(String callerId) {
    return Response.newSuccess("Success", master.getUri().toString()).toList();
  }

  @Override
  public List<Object> lookupNode(String callerId, String nodeName) {
    URI nodeSlaveUri = master.lookupNode(GraphName.of(nodeName));
    if (nodeSlaveUri != null) {
      return Response.newSuccess("Success", nodeSlaveUri.toString()).toList();
    } else {
      return Response.newError("No such node", null).toList();
    }
  }

  @Override
  public List<Object> registerPublisher(String callerId, String topicName, String topicMessageType,
      String callerSlaveUri) {
    try {
      List<URI> subscribers =
          master.registerPublisher(GraphName.of(callerId), new URI(callerSlaveUri), GraphName.of(
              topicName), topicMessageType);
      List<String> urls = Lists.newArrayList();
      for (URI uri : subscribers) {
        urls.add(uri.toString());
      }
      return Response.newSuccess("Success", urls).toList();
    } catch (URISyntaxException e) {
      throw new RosRuntimeException(String.format("Improperly formatted URI %s for publisher",
          callerSlaveUri), e);
    }
  }

  @Override
  public List<Object> unregisterPublisher(String callerId, String topicName, String callerSlaveUri) {
    boolean result = master.unregisterPublisher(GraphName.of(callerId), GraphName.of(topicName));
    return Response.newSuccess("Success", result ? 1 : 0).toList();
  }

  @Override
  public List<Object> registerSubscriber(String callerId, String topicName,
      String topicMessageType, String callerSlaveUri) {
    try {
      List<URI> publishers =
          master.registerSubscriber(GraphName.of(callerId), new URI(callerSlaveUri),
              GraphName.of(topicName), topicMessageType);
      List<String> urls = Lists.newArrayList();
      for (URI uri : publishers) {
        urls.add(uri.toString());
      }
      return Response.newSuccess("Success", urls).toList();
    } catch (URISyntaxException e) {
      throw new RosRuntimeException(String.format("Improperly formatted URI %s for subscriber",
          callerSlaveUri), e);
    }
  }

  @Override
  public List<Object>
      unregisterSubscriber(String callerId, String topicName, String callerSlaveUri) {
    boolean result = master.unregisterSubscriber(GraphName.of(callerId), GraphName.of(topicName));
    return Response.newSuccess("Success", result ? 1 : 0).toList();
  }

  @Override
  public List<Object> lookupService(String callerId, String serviceName) {
    URI slaveUri = master.lookupService(GraphName.of(serviceName));
    if (slaveUri != null) {
      return Response.newSuccess("Success", slaveUri.toString()).toList();
    }
    return Response.newError("No such service.", null).toList();
  }

  @Override
  public List<Object> registerService(String callerId, String serviceName, String serviceUri,
      String callerSlaveUri) {
    try {
      master.registerService(GraphName.of(callerId), new URI(callerSlaveUri), GraphName.of(
          serviceName), new URI(serviceUri));
      return Response.newSuccess("Success", 0).toList();
    } catch (URISyntaxException e) {
      throw new RosRuntimeException(e);
    }
  }

  @Override
  public List<Object> unregisterService(String callerId, String serviceName, String serviceUri) {
    try {
      boolean result =
          master.unregisterService(GraphName.of(callerId), GraphName.of(serviceName), new URI(
              serviceUri));
      return Response.newSuccess("Success", result ? 1 : 0).toList();
    } catch (URISyntaxException e) {
      throw new RosRuntimeException(e);
    }
  }

  @Override
  public List<Object> setParam(String callerId, String key, Boolean value) {
    parameterServer.set(GraphName.of(key), value);
    return Response.newSuccess("Success", null).toList();
  }

  @Override
  public List<Object> setParam(String callerId, String key, Integer value) {
    parameterServer.set(GraphName.of(key), value);
    return Response.newSuccess("Success", null).toList();
  }

  @Override
  public List<Object> setParam(String callerId, String key, Double value) {
    parameterServer.set(GraphName.of(key), value);
    return Response.newSuccess("Success", null).toList();
  }

  @Override
  public List<Object> setParam(String callerId, String key, String value) {
    parameterServer.set(GraphName.of(key), value);
    return Response.newSuccess("Success", null).toList();
  }

  @Override
  public List<Object> setParam(String callerId, String key, List<?> value) {
    parameterServer.set(GraphName.of(key), value);
    return Response.newSuccess("Success", null).toList();
  }

  @Override
  public List<Object> setParam(String callerId, String key, Map<?, ?> value) {
    parameterServer.set(GraphName.of(key), value);
    return Response.newSuccess("Success", null).toList();
  }

  @Override
  public List<Object> getParam(String callerId, String key) {
    Object value = parameterServer.get(GraphName.of(key));
    if (value == null) {
      return Response.newError("Parameter \"" + key + "\" is not set.", null).toList();
    }
    return Response.newSuccess("Success", value).toList();
  }

  @Override
  public List<Object> searchParam(String callerId, String key) {
    throw new UnsupportedOperationException();
  }

  @Override
  public List<Object> subscribeParam(String callerId, String callerSlaveUri, String key) {
    parameterServer.subscribe(GraphName.of(key),
        NodeIdentifier.forNameAndUri(callerId, callerSlaveUri));
    Object value = parameterServer.get(GraphName.of(key));
    if (value == null) {
      // Must return an empty map as the value of an unset parameter.
      value = new HashMap<String, Object>();
    }
    return Response.newSuccess("Success", value).toList();
  }

  @Override
  public List<Object> unsubscribeParam(String callerId, String callerSlaveUri, String key) {
    throw new UnsupportedOperationException();
  }

  @Override
  public List<Object> deleteParam(String callerId, String key) {
    parameterServer.delete(GraphName.of(key));
    return Response.newSuccess("Success", null).toList();
  }

  @Override
  public List<Object> hasParam(String callerId, String key) {
    return Response.newSuccess("Success", parameterServer.has(GraphName.of(key))).toList();
  }

  @Override
  public List<Object> getParamNames(String callerId) {
    Collection<GraphName> names = parameterServer.getNames();
    List<String> stringNames = Lists.newArrayList();
    for (GraphName name : names) {
      stringNames.add(name.toString());
    }
    return Response.newSuccess("Success", stringNames).toList();
  }
}
