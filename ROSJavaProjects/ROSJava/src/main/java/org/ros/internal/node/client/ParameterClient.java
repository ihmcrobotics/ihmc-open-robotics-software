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

package org.ros.internal.node.client;

import com.google.common.collect.Lists;

import org.ros.internal.node.response.BooleanResultFactory;
import org.ros.internal.node.response.IntegerResultFactory;
import org.ros.internal.node.response.ObjectResultFactory;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.StringListResultFactory;
import org.ros.internal.node.response.StringResultFactory;
import org.ros.internal.node.response.VoidResultFactory;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.server.ParameterServer;
import org.ros.internal.node.xmlrpc.ParameterServerXmlRpcEndpoint;
import org.ros.namespace.GraphName;

import java.net.URI;
import java.util.List;
import java.util.Map;

/**
 * Provide access to the XML-RPC API for a ROS {@link ParameterServer}.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ParameterClient extends Client<ParameterServerXmlRpcEndpoint> {

  private final NodeIdentifier nodeIdentifier;
  private final String nodeName;

  /**
   * Create a new {@link ParameterClient} connected to the specified
   * {@link ParameterServer} URI.
   * 
   * @param uri
   *          the {@link URI} of the {@link ParameterServer} to connect to
   */
  public ParameterClient(NodeIdentifier nodeIdentifier, URI uri) {
    super(uri, ParameterServerXmlRpcEndpoint.class);
    this.nodeIdentifier = nodeIdentifier;
    nodeName = nodeIdentifier.getName().toString();
  }

  public Response<Object> getParam(GraphName parameterName) {
    return Response.fromListCheckedFailure(xmlRpcEndpoint.getParam(nodeName, parameterName.toString()),
        new ObjectResultFactory());
  }

  public Response<Void> setParam(GraphName parameterName, Boolean parameterValue) {
    return Response.fromListChecked(
        xmlRpcEndpoint.setParam(nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
  }

  public Response<Void> setParam(GraphName parameterName, Integer parameterValue) {
    return Response.fromListChecked(
        xmlRpcEndpoint.setParam(nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
  }

  public Response<Void> setParam(GraphName parameterName, Double parameterValue) {
    return Response.fromListChecked(
        xmlRpcEndpoint.setParam(nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
  }

  public Response<Void> setParam(GraphName parameterName, String parameterValue) {
    return Response.fromListChecked(
        xmlRpcEndpoint.setParam(nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
  }

  public Response<Void> setParam(GraphName parameterName, List<?> parameterValue) {
    return Response.fromListChecked(
        xmlRpcEndpoint.setParam(nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
  }

  public Response<Void> setParam(GraphName parameterName, Map<?, ?> parameterValue) {
    return Response.fromListChecked(
        xmlRpcEndpoint.setParam(nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
  }

  public Response<GraphName> searchParam(GraphName parameterName) {
    Response<String> response =
        Response.fromListCheckedFailure(xmlRpcEndpoint.searchParam(nodeName, parameterName.toString()),
            new StringResultFactory());
    return new Response<GraphName>(response.getStatusCode(), response.getStatusMessage(),
        GraphName.of(response.getResult()));
  }

  public Response<Object> subscribeParam(GraphName parameterName) {
    return Response.fromListChecked(xmlRpcEndpoint.subscribeParam(nodeName, nodeIdentifier.getUri()
        .toString(), parameterName.toString()), new ObjectResultFactory());
  }

  public Response<Integer> unsubscribeParam(GraphName parameterName) {
    return Response.fromListChecked(
        xmlRpcEndpoint.unsubscribeParam(nodeName, nodeIdentifier.getUri().toString(),
            parameterName.toString()), new IntegerResultFactory());
  }

  public Response<Boolean> hasParam(GraphName parameterName) {
    return Response.fromListChecked(xmlRpcEndpoint.hasParam(nodeName, parameterName.toString()),
        new BooleanResultFactory());
  }

  public Response<Void> deleteParam(GraphName parameterName) {
    return Response.fromListChecked(xmlRpcEndpoint.deleteParam(nodeName, parameterName.toString()),
        new VoidResultFactory());
  }

  public Response<List<GraphName>> getParamNames() {
    Response<List<String>> response =
        Response.fromListChecked(xmlRpcEndpoint.getParamNames(nodeName), new StringListResultFactory());
    List<GraphName> graphNames = Lists.newArrayList();
    for (String name : response.getResult()) {
      graphNames.add(GraphName.of(name));
    }
    return new Response<List<GraphName>>(response.getStatusCode(), response.getStatusMessage(),
        graphNames);
  }
}
