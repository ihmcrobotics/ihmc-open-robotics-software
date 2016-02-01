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

/**
 * Remapping keys used to override ROS environment and other configuration
 * settings from a command-line-based executation of a node. As of ROS 1.4, only
 * {@code CommandLine.NODE_NAME} and {@code CommandLine.ROS_NAMESPACE} are
 * required.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 */
public interface CommandLineVariables {

  public static String ROS_NAMESPACE = "__ns";
  public static String ROS_IP = "__ip";
  public static String ROS_MASTER_URI = "__master";
  public static String TCPROS_PORT = "__tcpros_server_port";
  public static String NODE_NAME = "__name";
}
