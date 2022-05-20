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

package org.ros.node;

import org.ros.internal.node.parameter.DefaultParameterTree;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;

import java.net.URI;
import java.util.Collection;
import java.util.List;
import java.util.Map;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class AnonymousParmeterTree implements ParameterTree {

  private ParameterTree parameterTree;

  public AnonymousParmeterTree(URI masterUri) {
    NodeIdentifier nodeIdentifier = new NodeIdentifier(GraphName.of("invalid"), null);
    parameterTree =
        DefaultParameterTree.newFromNodeIdentifier(nodeIdentifier, masterUri, NameResolver.newRoot(), null);
  }

  @Override
  public boolean getBoolean(GraphName name) {
    return parameterTree.getBoolean(name);
  }

  @Override
  public boolean getBoolean(String name) {
    return parameterTree.getBoolean(name);
  }

  @Override
  public boolean getBoolean(GraphName name, boolean defaultValue) {
    return parameterTree.getBoolean(name, defaultValue);
  }

  @Override
  public boolean getBoolean(String name, boolean defaultValue) {
    return parameterTree.getBoolean(name, defaultValue);
  }

  @Override
  public int getInteger(GraphName name) {
    return parameterTree.getInteger(name);
  }

  @Override
  public int getInteger(String name) {
    return parameterTree.getInteger(name);
  }

  @Override
  public int getInteger(GraphName name, int defaultValue) {
    return parameterTree.getInteger(name, defaultValue);
  }

  @Override
  public int getInteger(String name, int defaultValue) {
    return parameterTree.getInteger(name, defaultValue);
  }

  @Override
  public double getDouble(GraphName name) {
    return parameterTree.getDouble(name);
  }

  @Override
  public double getDouble(String name) {
    return parameterTree.getDouble(name);
  }

  @Override
  public double getDouble(GraphName name, double defaultValue) {
    return parameterTree.getDouble(name, defaultValue);
  }

  @Override
  public double getDouble(String name, double defaultValue) {
    return parameterTree.getDouble(name, defaultValue);
  }

  @Override
  public String getString(GraphName name) {
    return parameterTree.getString(name);
  }

  @Override
  public String getString(String name) {
    return parameterTree.getString(name);
  }

  @Override
  public String getString(GraphName name, String defaultValue) {
    return parameterTree.getString(name, defaultValue);
  }

  @Override
  public String getString(String name, String defaultValue) {
    return parameterTree.getString(name, defaultValue);
  }

  @Override
  public List<?> getList(GraphName name) {
    return parameterTree.getList(name);
  }

  @Override
  public List<?> getList(String name) {
    return parameterTree.getList(name);
  }

  @Override
  public List<?> getList(GraphName name, List<?> defaultValue) {
    return parameterTree.getList(name, defaultValue);
  }

  @Override
  public List<?> getList(String name, List<?> defaultValue) {
    return parameterTree.getList(name, defaultValue);
  }

  @Override
  public Map<?, ?> getMap(GraphName name) {
    return parameterTree.getMap(name);
  }

  @Override
  public Map<?, ?> getMap(String name) {
    return parameterTree.getMap(name);
  }

  @Override
  public Map<?, ?> getMap(GraphName name, Map<?, ?> defaultValue) {
    return parameterTree.getMap(name, defaultValue);
  }

  @Override
  public Map<?, ?> getMap(String name, Map<?, ?> defaultValue) {
    return parameterTree.getMap(name, defaultValue);
  }

  @Override
  public void set(GraphName name, boolean value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(String name, boolean value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(GraphName name, int value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(String name, int value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(GraphName name, double value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(String name, double value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(GraphName name, String value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(String name, String value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(GraphName name, List<?> value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(String name, List<?> value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(GraphName name, Map<?, ?> value) {
    parameterTree.set(name, value);
  }

  @Override
  public void set(String name, Map<?, ?> value) {
    parameterTree.set(name, value);
  }

  @Override
  public boolean has(GraphName name) {
    return parameterTree.has(name);
  }

  @Override
  public boolean has(String name) {
    return parameterTree.has(name);
  }

  @Override
  public void delete(GraphName name) {
    parameterTree.delete(name);
  }

  @Override
  public void delete(String name) {
    parameterTree.delete(name);
  }

  @Override
  public GraphName search(GraphName name) {
    return parameterTree.search(name);
  }

  @Override
  public GraphName search(String name) {
    return parameterTree.search(name);
  }

  @Override
  public Collection<GraphName> getNames() {
    return parameterTree.getNames();
  }

  /**
   * @throws UnsupportedOperationException
   */
  @Override
  public void addParameterListener(GraphName name, ParameterListener listener) {
    throw new UnsupportedOperationException();
  }

  /**
   * @throws UnsupportedOperationException
   */
  @Override
  public void addParameterListener(String name, ParameterListener listener) {
    throw new UnsupportedOperationException();
  }
}
