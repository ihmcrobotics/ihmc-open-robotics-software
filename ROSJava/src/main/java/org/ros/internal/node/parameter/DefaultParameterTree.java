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

package org.ros.internal.node.parameter;

import java.net.URI;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import org.ros.exception.ParameterClassCastException;
import org.ros.exception.ParameterNotFoundException;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.StatusCode;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.server.ParameterServer;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;

import com.google.common.base.Preconditions;

/**
 * Provides access to the ROS {@link ParameterServer}.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class DefaultParameterTree implements ParameterTree {

  private final ParameterClient parameterClient;
  private final ParameterManager parameterManager;
  private final NameResolver resolver;

  public static DefaultParameterTree newFromNodeIdentifier(NodeIdentifier nodeIdentifier,
      URI masterUri, NameResolver resolver, ParameterManager parameterManager) {
    ParameterClient client = new ParameterClient(nodeIdentifier, masterUri);
    return new DefaultParameterTree(client, parameterManager, resolver);
  }

  private DefaultParameterTree(ParameterClient parameterClient, ParameterManager parameterManager,
      NameResolver resolver) {
    this.parameterClient = parameterClient;
    this.parameterManager = parameterManager;
    this.resolver = resolver;
  }

  
  public boolean has(GraphName name) {
    GraphName resolvedName = resolver.resolve(name);
    return parameterClient.hasParam(resolvedName).getResult();
  }

  
  public boolean has(String name) {
    return has(GraphName.of(name));
  }

  
  public void delete(GraphName name) {
    GraphName resolvedName = resolver.resolve(name);
    parameterClient.deleteParam(resolvedName);
  }

  
  public void delete(String name) {
    delete(GraphName.of(name));
  }

  
  public GraphName search(GraphName name) {
    GraphName resolvedName = resolver.resolve(name);
    Response<GraphName> response = parameterClient.searchParam(resolvedName);
    if (response.getStatusCode() == StatusCode.SUCCESS) {
      return response.getResult();
    } else {
      return null;
    }
  }

  
  public GraphName search(String name) {
    return search(GraphName.of(name));
  }

  
  public List<GraphName> getNames() {
    return parameterClient.getParamNames().getResult();
  }

  
  public void addParameterListener(GraphName name, ParameterListener listener) {
    parameterManager.addListener(name, listener);
    parameterClient.subscribeParam(name);
  }

  
  public void addParameterListener(String name, ParameterListener listener) {
    addParameterListener(GraphName.of(name), listener);
  }

  
  public void set(GraphName name, boolean value) {
    GraphName resolvedName = resolver.resolve(name);
    parameterClient.setParam(resolvedName, value);
  }

  
  public void set(String name, boolean value) {
    set(GraphName.of(name), value);
  }

  
  public void set(GraphName name, int value) {
    GraphName resolvedName = resolver.resolve(name);
    parameterClient.setParam(resolvedName, value);
  }

  
  public void set(String name, int value) {
    set(GraphName.of(name), value);
  }

  
  public void set(GraphName name, double value) {
    GraphName resolvedName = resolver.resolve(name);
    parameterClient.setParam(resolvedName, value);
  }

  
  public void set(String name, double value) {
    set(GraphName.of(name), value);
  }

  
  public void set(GraphName name, String value) {
    GraphName resolvedName = resolver.resolve(name);
    parameterClient.setParam(resolvedName, value);
  }

  
  public void set(String name, String value) {
    set(GraphName.of(name), value);
  }

  
  public void set(GraphName name, List<?> value) {
    GraphName resolvedName = resolver.resolve(name);
    parameterClient.setParam(resolvedName, value);
  }

  
  public void set(String name, List<?> value) {
    set(GraphName.of(name), value);
  }

  
  public void set(GraphName name, Map<?, ?> value) {
    GraphName resolvedName = resolver.resolve(name);
    parameterClient.setParam(resolvedName, value);
  }

  
  public void set(String name, Map<?, ?> value) {
    set(GraphName.of(name), value);
  }

  private <T> T get(GraphName name, Class<T> type) {
    GraphName resolvedName = resolver.resolve(name);
    Response<Object> response = parameterClient.getParam(resolvedName);
    try {
      if (response.getStatusCode() == StatusCode.SUCCESS) {
        return type.cast(response.getResult());
      }
    } catch (ClassCastException e) {
      throw new ParameterClassCastException("Cannot cast parameter to: " + type.getName(), e);
    }
    throw new ParameterNotFoundException("Parameter does not exist: " + name);
  }

  @SuppressWarnings("unchecked")
  private <T> T get(GraphName name, T defaultValue) {
    Preconditions.checkNotNull(defaultValue);
    GraphName resolvedName = resolver.resolve(name);
    Response<Object> response = parameterClient.getParam(resolvedName);
    if (response.getStatusCode() == StatusCode.SUCCESS) {
      try {
        return (T) defaultValue.getClass().cast(response.getResult());
      } catch (ClassCastException e) {
        throw new ParameterClassCastException("Cannot cast parameter to: "
            + defaultValue.getClass().getName(), e);
      }
    } else {
      return defaultValue;
    }
  }

  
  public boolean getBoolean(GraphName name) {
    return get(name, Boolean.class);
  }

  
  public boolean getBoolean(String name) {
    return getBoolean(GraphName.of(name));
  }

  
  public boolean getBoolean(GraphName name, boolean defaultValue) {
    return get(name, defaultValue);
  }

  
  public boolean getBoolean(String name, boolean defaultValue) {
    return getBoolean(GraphName.of(name), defaultValue);
  }

  
  public int getInteger(GraphName name) {
    return get(name, Integer.class);
  }

  
  public int getInteger(String name) {
    return getInteger(GraphName.of(name));
  }

  
  public int getInteger(GraphName name, int defaultValue) {
    return get(name, defaultValue);
  }

  
  public int getInteger(String name, int defaultValue) {
    return getInteger(GraphName.of(name), defaultValue);
  }

  
  public double getDouble(GraphName name) {
    return get(name, Double.class);
  }

  
  public double getDouble(String name) {
    return getDouble(GraphName.of(name));
  }

  
  public double getDouble(GraphName name, double defaultValue) {
    return get(name, defaultValue);
  }

  
  public double getDouble(String name, double defaultValue) {
    return getDouble(GraphName.of(name), defaultValue);
  }

  
  public String getString(GraphName name) {
    return get(name, String.class);
  }

  
  public String getString(String name) {
    return get(GraphName.of(name), String.class);
  }

  
  public String getString(GraphName name, String defaultValue) {
    return get(name, defaultValue);
  }

  
  public String getString(String name, String defaultValue) {
    return getString(GraphName.of(name), defaultValue);
  }

  
  public List<?> getList(GraphName name) {
    return Arrays.asList(get(name, Object[].class));
  }

  
  public List<?> getList(String name) {
    return getList(GraphName.of(name));
  }

  
  public List<?> getList(GraphName name, List<?> defaultValue) {
    return Arrays.asList(get(name, defaultValue.toArray()));
  }

  
  public List<?> getList(String name, List<?> defaultValue) {
    return getList(GraphName.of(name), defaultValue);
  }

  
  public Map<?, ?> getMap(GraphName name) {
    return get(name, Map.class);
  }

  
  public Map<?, ?> getMap(String name) {
    return getMap(GraphName.of(name));
  }

  
  public Map<?, ?> getMap(GraphName name, Map<?, ?> defaultValue) {
    return get(name, defaultValue);
  }

  
  public Map<?, ?> getMap(String name, Map<?, ?> defaultValue) {
    return getMap(GraphName.of(name), defaultValue);
  }
}
