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

package org.ros.node.parameter;

import org.ros.exception.ParameterClassCastException;
import org.ros.exception.ParameterNotFoundException;
import org.ros.internal.node.server.ParameterServer;
import org.ros.namespace.GraphName;

import java.util.Collection;
import java.util.List;
import java.util.Map;

/**
 * Provides access to a {@link ParameterServer}.
 * 
 * <p>
 * A parameter server is a shared, multi-variate dictionary that is accessible
 * via network APIs. Nodes use this server to store and retrieve parameters at
 * runtime. As it is not designed for high-performance, it is best used for
 * static, non-binary data such as configuration parameters. It is meant to be
 * globally viewable so that tools can easily inspect the configuration state of
 * the system and modify if necessary.
 * 
 * @see <a href="http://www.ros.org/wiki/Parameter%20Server">Parameter server
 *      documentation</a>
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface ParameterTree {

  /**
   * @param name
   *          the parameter name
   * @return the parameter value
   * @throws ParameterNotFoundException
   *           if the parameter is not found
   * @throws ParameterClassCastException
   *           if the parameter is not the expected type
   */
  boolean getBoolean(GraphName name);

  /**
   * @see #getBoolean(GraphName)
   */
  boolean getBoolean(String name);

  /**
   * @param name
   *          the parameter name
   * @param defaultValue
   *          the default value
   * @return the parameter value or the default value if the parameter does not
   *         exist
   * @throws ParameterClassCastException
   *           if the parameter exists and is not the expected type
   */
  boolean getBoolean(GraphName name, boolean defaultValue);

  /**
   * @see #getBoolean(GraphName, boolean)
   */
  boolean getBoolean(String name, boolean defaultValue);

  /**
   * @param name
   *          the parameter name
   * @return the parameter value
   * @throws ParameterNotFoundException
   *           if the parameter is not found
   * @throws ParameterClassCastException
   *           if the parameter is not the expected type
   */
  int getInteger(GraphName name);

  /**
   * @see #getInteger(GraphName)
   */
  int getInteger(String name);

  /**
   * @param name
   *          the parameter name
   * @param defaultValue
   *          the default value
   * @return the parameter value or the default value if the parameter does not
   *         exist
   * @throws ParameterClassCastException
   *           if the parameter exists and is not the expected type
   */
  int getInteger(GraphName name, int defaultValue);

  /**
   * @see #getInteger(GraphName, int)
   */
  int getInteger(String name, int defaultValue);

  /**
   * @param name
   *          the parameter name
   * @return the parameter value
   * @throws ParameterNotFoundException
   *           if the parameter is not found
   * @throws ParameterClassCastException
   *           if the parameter is not the expected type
   */
  double getDouble(GraphName name);

  /**
   * @see #getDouble(GraphName)
   */
  double getDouble(String name);

  /**
   * @param name
   *          the parameter name
   * @param defaultValue
   *          the default value
   * @return the parameter value or the default value if the parameter does not
   *         exist
   * @throws ParameterClassCastException
   *           if the parameter exists and is not the expected type
   */
  double getDouble(GraphName name, double defaultValue);

  /**
   * @see #getDouble(GraphName, double)
   */
  double getDouble(String name, double defaultValue);

  /**
   * @param name
   *          the parameter name
   * @return the parameter value:w
   * 
   * @throws ParameterNotFoundException
   *           if the parameter is not found
   * @throws ParameterClassCastException
   *           if the parameter is not the expected type
   */
  String getString(GraphName name);

  /**
   * @see #getString(GraphName)
   */
  String getString(String name);

  /**
   * @param name
   *          the parameter name
   * @param defaultValue
   *          the default value
   * @return the parameter value or the default value if the parameter does not
   *         exist
   * @throws ParameterClassCastException
   *           if the parameter exists and is not the expected type
   */
  String getString(GraphName name, String defaultValue);

  /**
   * @see #getString(GraphName, String)
   */
  String getString(String name, String defaultValue);

  /**
   * @param name
   *          the parameter name
   * @return the parameter value
   * @throws ParameterNotFoundException
   *           if the parameter is not found
   * @throws ParameterClassCastException
   *           if the parameter is not the expected type
   */
  List<?> getList(GraphName name);

  /**
   * @see #getList(GraphName)
   */
  List<?> getList(String name);

  /**
   * @param name
   *          the parameter name
   * @param defaultValue
   *          the default value
   * @return the parameter value or the default value if the parameter does not
   *         exist
   * @throws ParameterClassCastException
   *           if the parameter exists and is not the expected type
   */
  List<?> getList(GraphName name, List<?> defaultValue);

  /**
   * @see #getList(GraphName, List)
   */
  List<?> getList(String name, List<?> defaultValue);

  /**
   * @param name
   *          the parameter name
   * @return the parameter value
   * @throws ParameterNotFoundException
   *           if the parameter is not found
   * @throws ParameterClassCastException
   *           if the parameter is not the expected type
   */
  Map<?, ?> getMap(GraphName name);

  /**
   * @see #getMap(GraphName)
   */
  Map<?, ?> getMap(String name);

  /**
   * @param name
   *          the parameter name
   * @param defaultValue
   *          the default value
   * @return the parameter value or the default value if the parameter does not
   *         exist
   * @throws ParameterClassCastException
   *           if the parameter exists and is not the expected type
   */
  Map<?, ?> getMap(GraphName name, Map<?, ?> defaultValue);

  /**
   * @see #getMap(GraphName, Map)
   */
  Map<?, ?> getMap(String name, Map<?, ?> defaultValue);

  /**
   * @param name
   *          the parameter name
   * @param value
   *          the value that the parameter will be set to
   */
  void set(GraphName name, boolean value);

  /**
   * @see #set(GraphName, boolean)
   */
  void set(String name, boolean value);

  /**
   * @param name
   *          the parameter name
   * @param value
   *          the value that the parameter will be set to
   */
  void set(GraphName name, int value);

  /**
   * @see #set(GraphName, int)
   */
  void set(String name, int value);

  /**
   * @param name
   *          the parameter name
   * @param value
   *          the value that the parameter will be set to
   */
  void set(GraphName name, double value);

  /**
   * @see #set(GraphName, double)
   */
  void set(String name, double value);

  /**
   * @param name
   *          the parameter name
   * @param value
   *          the value that the parameter will be set to
   */
  void set(GraphName name, String value);

  /**
   * @see #set(GraphName, String)
   */
  void set(String name, String value);

  /**
   * @param name
   *          the parameter name
   * @param value
   *          the value that the parameter will be set to
   */
  void set(GraphName name, List<?> value);

  /**
   * @see #set(GraphName, List)
   */
  void set(String name, List<?> value);

  /**
   * @param name
   *          the parameter name
   * @param value
   *          the value that the parameter will be set to
   */
  void set(GraphName name, Map<?, ?> value);

  /**
   * @see #set(GraphName, Map)
   */
  void set(String name, Map<?, ?> value);

  /**
   * @param name
   *          the parameter name
   * @return {@code true} if a parameter with the given name exists,
   *         {@code false} otherwise
   */
  boolean has(GraphName name);

  /**
   * @see #has(GraphName)
   */
  boolean has(String name);

  /**
   * Deletes a specified parameter.
   * 
   * @param name
   *          the parameter name
   */
  void delete(GraphName name);

  /**
   * @see #delete(GraphName)
   */
  void delete(String name);

  /**
   * Search for parameter key on the Parameter Server. Search starts in caller's
   * namespace and proceeds upwards through parent namespaces until the
   * {@link ParameterServer} finds a matching key.
   * 
   * @param name
   *          the parameter name to search for
   * @return the name of the found parameter or {@code null} if no matching
   *         parameter was found
   */
  GraphName search(GraphName name);

  /**
   * @see #search(GraphName)
   */
  GraphName search(String name);

  /**
   * @return all known parameter names
   */
  Collection<GraphName> getNames();

  /**
   * Subscribes to changes to the specified parameter.
   * 
   * @param name
   *          the parameter name to subscribe to
   * @param listener
   *          a {@link ParameterListener} that will be called when the
   *          subscribed parameter changes
   */
  void addParameterListener(GraphName name, ParameterListener listener);

  /**
   * @see #addParameterListener(GraphName, ParameterListener)
   */
  void addParameterListener(String name, ParameterListener listener);
}
