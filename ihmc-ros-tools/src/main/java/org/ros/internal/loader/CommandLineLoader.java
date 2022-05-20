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

package org.ros.internal.loader;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import org.ros.CommandLineVariables;
import org.ros.EnvironmentVariables;
import org.ros.address.InetAddressFactory;
import org.ros.exception.RosRuntimeException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Create {@link NodeConfiguration} instances using a ROS command-line and
 * environment specification.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class CommandLineLoader {

  private final List<String> argv;
  private final List<String> nodeArguments;
  private final List<String> remappingArguments;
  private final Map<String, String> environment;
  private final Map<String, String> specialRemappings;
  private final Map<GraphName, GraphName> remappings;

  private String nodeClassName;

  /**
   * Create new {@link CommandLineLoader} with specified command-line arguments.
   * Environment variables will be pulled from default {@link System}
   * environment variables.
   * 
   * @param argv
   *          command-line arguments
   */
  public CommandLineLoader(List<String> argv) {
    this(argv, System.getenv());
  }

  /**
   * Create new {@link CommandLineLoader} with specified command-line arguments
   * and environment variables.
   * 
   * @param argv
   *          command-line arguments
   * @param environment
   *          environment variables
   */
  public CommandLineLoader(List<String> argv, Map<String, String> environment) {
    Preconditions.checkArgument(argv.size() > 0);
    this.argv = argv;
    this.environment = environment;
    nodeArguments = Lists.newArrayList();
    remappingArguments = Lists.newArrayList();
    remappings = Maps.newHashMap();
    specialRemappings = Maps.newHashMap();
    parseArgv();
  }

  private void parseArgv() {
    nodeClassName = argv.get(0);
    for (String argument : argv.subList(1, argv.size())) {
      if (argument.contains(":=")) {
        remappingArguments.add(argument);
      } else {
        nodeArguments.add(argument);
      }
    }
  }

  public String getNodeClassName() {
    return nodeClassName;
  }

  public List<String> getNodeArguments() {
    return Collections.unmodifiableList(nodeArguments);
  }

  /**
   * Create NodeConfiguration according to ROS command-line and environment
   * specification.
   */
  public NodeConfiguration build() {
    parseRemappingArguments();
    // TODO(damonkohler): Add support for starting up a private node.
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getHost());
    nodeConfiguration.setParentResolver(buildParentResolver());
    nodeConfiguration.setRosRoot(getRosRoot());
    nodeConfiguration.setRosPackagePath(getRosPackagePath());
    nodeConfiguration.setMasterUri(getMasterUri());
    if (specialRemappings.containsKey(CommandLineVariables.NODE_NAME)) {
      nodeConfiguration.setNodeName(specialRemappings.get(CommandLineVariables.NODE_NAME));
    }
    return nodeConfiguration;
  }

  private void parseRemappingArguments() {
    for (String remapping : remappingArguments) {
      Preconditions.checkState(remapping.contains(":="));
      String[] remap = remapping.split(":=");
      if (remap.length > 2) {
        throw new IllegalArgumentException("Invalid remapping argument: " + remapping);
      }
      if (remapping.startsWith("__")) {
        specialRemappings.put(remap[0], remap[1]);
      } else {
        remappings.put(GraphName.of(remap[0]), GraphName.of(remap[1]));
      }
    }
  }

  /**
   * Precedence:
   * 
   * <ol>
   * <li>The __ns:= command line argument.</li>
   * <li>The ROS_NAMESPACE environment variable.</li>
   * </ol>
   */
  private NameResolver buildParentResolver() {
    GraphName namespace = GraphName.root();
    if (specialRemappings.containsKey(CommandLineVariables.ROS_NAMESPACE)) {
      namespace =
          GraphName.of(specialRemappings.get(CommandLineVariables.ROS_NAMESPACE)).toGlobal();
    } else if (environment.containsKey(EnvironmentVariables.ROS_NAMESPACE)) {
      namespace = GraphName.of(environment.get(EnvironmentVariables.ROS_NAMESPACE)).toGlobal();
    }
    return new NameResolver(namespace, remappings);
  }

  /**
   * Precedence (default: null):
   * 
   * <ol>
   * <li>The __ip:= command line argument.</li>
   * <li>The ROS_IP environment variable.</li>
   * <li>The ROS_HOSTNAME environment variable.</li>
   * <li>The default host as specified in {@link NodeConfiguration}.</li>
   * </ol>
   */
  private String getHost() {
    String host = InetAddressFactory.newLoopback().getHostAddress();
    if (specialRemappings.containsKey(CommandLineVariables.ROS_IP)) {
      host = specialRemappings.get(CommandLineVariables.ROS_IP);
    } else if (environment.containsKey(EnvironmentVariables.ROS_IP)) {
      host = environment.get(EnvironmentVariables.ROS_IP);
    } else if (environment.containsKey(EnvironmentVariables.ROS_HOSTNAME)) {
      host = environment.get(EnvironmentVariables.ROS_HOSTNAME);
    }
    return host;
  }

  /**
   * Precedence:
   * 
   * <ol>
   * <li>The __master:= command line argument. This is not required but easy to
   * support.</li>
   * <li>The ROS_MASTER_URI environment variable.</li>
   * <li>The default master URI as defined in {@link NodeConfiguration}.</li>
   * </ol>
   */
  private URI getMasterUri() {
    URI uri = NodeConfiguration.DEFAULT_MASTER_URI;
    try {
      if (specialRemappings.containsKey(CommandLineVariables.ROS_MASTER_URI)) {
        uri = new URI(specialRemappings.get(CommandLineVariables.ROS_MASTER_URI));
      } else if (environment.containsKey(EnvironmentVariables.ROS_MASTER_URI)) {
        uri = new URI(environment.get(EnvironmentVariables.ROS_MASTER_URI));
      }
      return uri;
    } catch (URISyntaxException e) {
      throw new RosRuntimeException("Invalid master URI: " + uri);
    }
  }

  private File getRosRoot() {
    if (environment.containsKey(EnvironmentVariables.ROS_ROOT)) {
      return new File(environment.get(EnvironmentVariables.ROS_ROOT));
    } else {
      // For now, this is not required as we are not doing anything (e.g.
      // ClassLoader) that requires it. In the future, this may become required.
      return null;
    }
  }

  private List<File> getRosPackagePath() {
    if (environment.containsKey(EnvironmentVariables.ROS_PACKAGE_PATH)) {
      String rosPackagePath = environment.get(EnvironmentVariables.ROS_PACKAGE_PATH);
      List<File> paths = Lists.newArrayList();
      for (String path : rosPackagePath.split(File.pathSeparator)) {
        paths.add(new File(path));
      }
      return paths;
    } else {
      return Lists.newArrayList();
    }
  }

  /**
   * @param name
   *          the name of the class
   * @return an instance of {@link NodeMain}
   * @throws ClassNotFoundException
   * @throws InstantiationException
   * @throws IllegalAccessException
   */
  public NodeMain loadClass(String name) throws ClassNotFoundException, InstantiationException,
      IllegalAccessException {
    Class<?> clazz = getClass().getClassLoader().loadClass(name);
    return NodeMain.class.cast(clazz.newInstance());
  }
}
