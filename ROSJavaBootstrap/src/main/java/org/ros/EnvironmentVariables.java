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
 * ROS environment variables.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 */
public interface EnvironmentVariables {

  public static String ROS_MASTER_URI = "ROS_MASTER_URI";
  public static String ROS_IP = "ROS_IP";
  public static String ROS_HOSTNAME = "ROS_HOSTNAME";
  public static String ROS_NAMESPACE = "ROS_NAMESPACE";
  public static String ROS_ROOT = "ROS_ROOT";
  public static String ROS_PACKAGE_PATH = "ROS_PACKAGE_PATH";
}
