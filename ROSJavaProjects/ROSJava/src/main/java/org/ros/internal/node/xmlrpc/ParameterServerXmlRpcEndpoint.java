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

import org.ros.internal.node.server.ParameterServer;

import java.util.Collection;
import java.util.List;
import java.util.Map;

/**
 * XML-RPC endpoint for a parameter server.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface ParameterServerXmlRpcEndpoint extends XmlRpcEndpoint {

  /**
   * Deletes a parameter.
   * 
   * @param callerId
   *          ROS caller ID
   * @param key
   *          parameter name
   * @return void
   */
  public List<Object> deleteParam(String callerId, String key);

  /**
   * Sets a parameter.
   * 
   * <p>
   * NOTE: if value is a dictionary it will be treated as a parameter tree,
   * where key is the parameter namespace. For example
   * {'x':1,'y':2,'sub':{'z':3}} will set key/x=1, key/y=2, and key/sub/z=3.
   * Furthermore, it will replace all existing parameters in the key parameter
   * namespace with the parameters in value. You must set parameters
   * individually if you wish to perform a union update.
   * 
   * @param callerId
   *          ROS caller ID
   * @param key
   *          Parameter name.
   * @param value
   *          Parameter value.
   * @return void
   */
  public List<Object> setParam(String callerId, String key, Boolean value);

  public List<Object> setParam(String callerId, String key, Integer value);

  public List<Object> setParam(String callerId, String key, Double value);

  public List<Object> setParam(String callerId, String key, String value);

  public List<Object> setParam(String callerId, String key, List<?> value);

  public List<Object> setParam(String callerId, String key, Map<?, ?> value);

  /**
   * Retrieve parameter value from server.
   * 
   * <p>
   * If code is not 1, parameterValue should be ignored. If key is a namespace,
   * the return value will be a dictionary, where each key is a parameter in
   * that namespace. Sub-namespaces are also represented as dictionaries.
   * 
   * @param callerId
   *          ROS caller ID
   * @param key
   *          Parameter name. If key is a namespace, getParam() will return a
   *          parameter tree.
   * @return the parameter value
   */
  public List<Object> getParam(String callerId, String key);

  /**
   * Searches for a parameter key on the {@link ParameterServer}.
   * 
   * <p>
   * Search starts in caller's namespace and proceeds upwards through parent
   * namespaces until Parameter Server finds a matching key. searchParam()'s
   * behavior is to search for the first partial match. For example, imagine
   * that there are two 'robot_description' parameters /robot_description
   * /robot_description/arm /robot_description/base /pr2/robot_description
   * /pr2/robot_description/base If I start in the namespace /pr2/foo and search
   * for robot_description, searchParam() will match /pr2/robot_description. If
   * I search for robot_description/arm it will return
   * /pr2/robot_description/arm, even though that parameter does not exist
   * (yet).
   * 
   * If code is not 1, foundKey should be ignored.
   * 
   * @param callerId
   *          ROS caller ID
   * @param key
   *          Parameter name to search for.
   * @return the found key
   */
  public List<Object> searchParam(String callerId, String key);

  /**
   * Retrieves the parameter value from server and subscribe to updates to that
   * param. See paramUpdate() in the Node API.
   * 
   * <p>
   * If code is not 1, parameterValue should be ignored. parameterValue is an
   * empty dictionary if the parameter has not been set yet.
   * 
   * @param callerId
   *          ROS caller ID
   * @param callerApi
   *          Node API URI of subscriber for paramUpdate callbacks.
   * @param key
   * @return the parameter value
   */
  public List<Object> subscribeParam(String callerId, String callerApi, String key);

  /**
   * Unsubscribes from updates to the specified param. See paramUpdate() in the
   * Node API.
   * 
   * <p>
   * A return value of zero means that the caller was not subscribed to the
   * parameter.
   * 
   * @param callerId
   *          ROS caller ID
   * @param callerApi
   *          Node API URI of subscriber
   * @param key
   *          Parameter name
   * @return the number of parameters that were unsubscribed
   */
  public List<Object> unsubscribeParam(String callerId, String callerApi, String key);

  /**
   * Check if parameter is stored on server.
   * 
   * @param callerId
   *          ROS caller ID.
   * @param key
   *          Parameter name.
   * @return {@code true} if the parameter exists
   */
  public List<Object> hasParam(String callerId, String key);

  /**
   * Gets the list of all parameter names stored on this server.
   * 
   * @param callerId
   *          ROS caller ID.
   * @return a {@link Collection} of parameter names
   */
  public List<Object> getParamNames(String callerId);

}