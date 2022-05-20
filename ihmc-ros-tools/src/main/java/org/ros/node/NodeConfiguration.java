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

import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;

import org.ros.address.AdvertiseAddress;
import org.ros.address.AdvertiseAddressFactory;
import org.ros.address.BindAddress;
import org.ros.address.PrivateAdvertiseAddressFactory;
import org.ros.address.PublicAdvertiseAddressFactory;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.DefaultMessageSerializationFactory;
import org.ros.internal.message.service.ServiceDescriptionFactory;
import org.ros.internal.message.service.ServiceRequestMessageFactory;
import org.ros.internal.message.service.ServiceResponseMessageFactory;
import org.ros.internal.message.topic.TopicDescriptionFactory;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializationFactory;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.time.TimeProvider;
import org.ros.time.WallTimeProvider;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;

/**
 * Stores configuration information (e.g. ROS master URI) for {@link Node}s.
 * 
 * @see <a href="http://www.ros.org/wiki/ROS/Technical%20Overview#Node">Node
 *      documentation</a>
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class NodeConfiguration {

  /**
   * The default master {@link URI}.
   */
  public static final URI DEFAULT_MASTER_URI;

  static {
    try {
      DEFAULT_MASTER_URI = new URI("http://localhost:11311/");
    } catch (URISyntaxException e) {
      throw new RosRuntimeException(e);
    }
  }

  private NameResolver parentResolver;
  private URI masterUri;
  private File rosRoot;
  private List<File> rosPackagePath;
  private GraphName nodeName;
  private TopicDescriptionFactory topicDescriptionFactory;
  private MessageFactory topicMessageFactory;
  private ServiceDescriptionFactory serviceDescriptionFactory;
  private MessageFactory serviceRequestMessageFactory;
  private MessageFactory serviceResponseMessageFactory;
  private MessageSerializationFactory messageSerializationFactory;
  private BindAddress tcpRosBindAddress;
  private AdvertiseAddressFactory tcpRosAdvertiseAddressFactory;
  private BindAddress xmlRpcBindAddress;
  private AdvertiseAddressFactory xmlRpcAdvertiseAddressFactory;
  private ScheduledExecutorService scheduledExecutorService;
  private TimeProvider timeProvider;

  /**
   * @param nodeConfiguration
   *          the {@link NodeConfiguration} to copy
   * @return a copy of the supplied {@link NodeConfiguration}
   */
  public static NodeConfiguration copyOf(NodeConfiguration nodeConfiguration) {
    NodeConfiguration copy = new NodeConfiguration();
    copy.parentResolver = nodeConfiguration.parentResolver;
    copy.masterUri = nodeConfiguration.masterUri;
    copy.rosRoot = nodeConfiguration.rosRoot;
    copy.rosPackagePath = nodeConfiguration.rosPackagePath;
    copy.nodeName = nodeConfiguration.nodeName;
    copy.topicDescriptionFactory = nodeConfiguration.topicDescriptionFactory;
    copy.topicMessageFactory = nodeConfiguration.topicMessageFactory;
    copy.serviceDescriptionFactory = nodeConfiguration.serviceDescriptionFactory;
    copy.serviceRequestMessageFactory = nodeConfiguration.serviceRequestMessageFactory;
    copy.serviceResponseMessageFactory = nodeConfiguration.serviceResponseMessageFactory;
    copy.messageSerializationFactory = nodeConfiguration.messageSerializationFactory;
    copy.tcpRosBindAddress = nodeConfiguration.tcpRosBindAddress;
    copy.tcpRosAdvertiseAddressFactory = nodeConfiguration.tcpRosAdvertiseAddressFactory;
    copy.xmlRpcBindAddress = nodeConfiguration.xmlRpcBindAddress;
    copy.xmlRpcAdvertiseAddressFactory = nodeConfiguration.xmlRpcAdvertiseAddressFactory;
    copy.scheduledExecutorService = nodeConfiguration.scheduledExecutorService;
    copy.timeProvider = nodeConfiguration.timeProvider;
    return copy;
  }

  /**
   * Creates a new {@link NodeConfiguration} for a publicly accessible
   * {@link Node}.
   * 
   * @param host
   *          the host that the {@link Node} will run on
   * @param masterUri
   *          the {@link URI} for the master that the {@link Node} will register
   *          with
   * @return a new {@link NodeConfiguration} for a publicly accessible
   *         {@link Node}
   */
  public static NodeConfiguration newPublic(String host, URI masterUri) {
    NodeConfiguration configuration = new NodeConfiguration();
    configuration.setXmlRpcBindAddress(BindAddress.newPublic());
    configuration.setXmlRpcAdvertiseAddressFactory(new PublicAdvertiseAddressFactory(host));
    configuration.setTcpRosBindAddress(BindAddress.newPublic());
    configuration.setTcpRosAdvertiseAddressFactory(new PublicAdvertiseAddressFactory(host));
    configuration.setMasterUri(masterUri);
    return configuration;
  }

  /**
   * Creates a new {@link NodeConfiguration} for a publicly accessible
   * {@link Node}.
   * 
   * @param host
   *          the host that the {@link Node} will run on
   * @return a new {@link NodeConfiguration} for a publicly accessible
   *         {@link Node}
   */
  public static NodeConfiguration newPublic(String host) {
    return newPublic(host, DEFAULT_MASTER_URI);
  }

  /**
   * Creates a new {@link NodeConfiguration} for a {@link Node} that is only
   * accessible on the local host.
   * 
   * @param masterUri
   *          the {@link URI} for the master that the {@link Node} will register
   *          with
   * @return a new {@link NodeConfiguration} for a private {@link Node}
   */
  public static NodeConfiguration newPrivate(URI masterUri) {
    NodeConfiguration configuration = new NodeConfiguration();
    configuration.setXmlRpcBindAddress(BindAddress.newPrivate());
    configuration.setXmlRpcAdvertiseAddressFactory(new PrivateAdvertiseAddressFactory());
    configuration.setTcpRosBindAddress(BindAddress.newPrivate());
    configuration.setTcpRosAdvertiseAddressFactory(new PrivateAdvertiseAddressFactory());
    configuration.setMasterUri(masterUri);
    return configuration;
  }

  /**
   * Creates a new {@link NodeConfiguration} for a {@link Node} that is only
   * accessible on the local host.
   * 
   * @return a new {@link NodeConfiguration} for a private {@link Node}
   */
  public static NodeConfiguration newPrivate() {
    return newPrivate(DEFAULT_MASTER_URI);
  }

  private NodeConfiguration() {
    MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    setTopicDescriptionFactory(new TopicDescriptionFactory(messageDefinitionProvider));
    setTopicMessageFactory(new DefaultMessageFactory(messageDefinitionProvider));
    setServiceDescriptionFactory(new ServiceDescriptionFactory(messageDefinitionProvider));
    setServiceRequestMessageFactory(new ServiceRequestMessageFactory(messageDefinitionProvider));
    setServiceResponseMessageFactory(new ServiceResponseMessageFactory(messageDefinitionProvider));
    setMessageSerializationFactory(new DefaultMessageSerializationFactory(messageDefinitionProvider));
    setParentResolver(NameResolver.newRoot());
    setTimeProvider(new WallTimeProvider());
  }

  /**
   * @return the {@link NameResolver} for the {@link Node}'s parent namespace
   */
  public NameResolver getParentResolver() {
    return parentResolver;
  }

  /**
   * @param resolver
   *          the {@link NameResolver} for the {@link Node}'s parent namespace
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setParentResolver(NameResolver resolver) {
    this.parentResolver = resolver;
    return this;
  }

  /**
   * @see <a
   *      href="http://www.ros.org/wiki/ROS/EnvironmentVariables#ROS_MASTER_URI">ROS_MASTER_URI
   *      documentation</a>
   * @return the {@link URI} of the master that the {@link Node} will register
   *         with
   */
  public URI getMasterUri() {
    return masterUri;
  }

  /**
   * @see <a
   *      href="http://www.ros.org/wiki/ROS/EnvironmentVariables#ROS_MASTER_URI">ROS_MASTER_URI
   *      documentation</a>
   * @param masterUri
   *          the {@link URI} of the master that the {@link Node} will register
   *          with
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setMasterUri(URI masterUri) {
    this.masterUri = masterUri;
    return this;
  }

  /**
   * @see <a
   *      href="http://www.ros.org/wiki/ROS/EnvironmentVariables#ROS_ROOT">ROS_ROOT
   *      documentation</a>
   * @return the location where the ROS core packages are installed
   */
  public File getRosRoot() {
    return rosRoot;
  }

  /**
   * @see <a
   *      href="http://www.ros.org/wiki/ROS/EnvironmentVariables#ROS_ROOT">ROS_ROOT
   *      documentation</a>
   * @param rosRoot
   *          the location where the ROS core packages are installed
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setRosRoot(File rosRoot) {
    this.rosRoot = rosRoot;
    return this;
  }

  /**
   * These ordered paths tell the ROS system where to search for more ROS
   * packages. If there are multiple packages of the same name, ROS will choose
   * the one that appears in the {@link List} first.
   * 
   * @see <a
   *      href="http://www.ros.org/wiki/ROS/EnvironmentVariables#ROS_PACKAGE_PATH">ROS_PACKAGE_PATH
   *      documentation</a>
   * @return the {@link List} of paths where the system will look for ROS
   *         packages
   */
  public List<File> getRosPackagePath() {
    return rosPackagePath;
  }

  /**
   * These ordered paths tell the ROS system where to search for more ROS
   * packages. If there are multiple packages of the same name, ROS will choose
   * the one that appears in the {@link List} first.
   * 
   * @see <a
   *      href="http://www.ros.org/wiki/ROS/EnvironmentVariables#ROS_PACKAGE_PATH">ROS_PACKAGE_PATH
   *      documentation</a>
   * @param rosPackagePath
   *          the {@link List} of paths where the system will look for ROS
   *          packages
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setRosPackagePath(List<File> rosPackagePath) {
    this.rosPackagePath = rosPackagePath;
    return this;
  }

  /**
   * @return the name of the {@link Node}
   */
  public GraphName getNodeName() {
    return nodeName;
  }

  /**
   * @param nodeName
   *          the name of the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setNodeName(GraphName nodeName) {
    this.nodeName = nodeName;
    return this;
  }

  /**
   * @param nodeName
   *          the name of the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setNodeName(String nodeName) {
    return setNodeName(GraphName.of(nodeName));
  }

  /**
   * Sets the name of the {@link Node} if the name has not already been set.
   * 
   * @param nodeName
   *          the name of the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setDefaultNodeName(GraphName nodeName) {
    if (this.nodeName == null) {
      setNodeName(nodeName);
    }
    return this;
  }

  /**
   * Sets the name of the {@link Node} if the name has not already been set.
   * 
   * @param nodeName
   *          the name of the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setDefaultNodeName(String nodeName) {
    return setDefaultNodeName(GraphName.of(nodeName));
  }

  /**
   * @return the {@link MessageSerializationFactory} for the {@link Node}
   */
  public MessageSerializationFactory getMessageSerializationFactory() {
    return messageSerializationFactory;
  }

  /**
   * @param messageSerializationFactory
   *          the {@link MessageSerializationFactory} for the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setMessageSerializationFactory(
      MessageSerializationFactory messageSerializationFactory) {
    this.messageSerializationFactory = messageSerializationFactory;
    return this;
  }

  /**
   * @param topicMessageFactory
   *          the {@link MessageFactory} for the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setTopicMessageFactory(MessageFactory topicMessageFactory) {
    this.topicMessageFactory = topicMessageFactory;
    return this;
  }

  public MessageFactory getTopicMessageFactory() {
    return topicMessageFactory;
  }

  /**
   * @param serviceRequestMessageFactory
   *          the {@link ServiceRequestMessageFactory} for the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setServiceRequestMessageFactory(
      ServiceRequestMessageFactory serviceRequestMessageFactory) {
    this.serviceRequestMessageFactory = serviceRequestMessageFactory;
    return this;
  }

  public MessageFactory getServiceRequestMessageFactory() {
    return serviceRequestMessageFactory;
  }

  /**
   * @param serviceResponseMessageFactory
   *          the {@link ServiceResponseMessageFactory} for the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setServiceResponseMessageFactory(
      ServiceResponseMessageFactory serviceResponseMessageFactory) {
    this.serviceResponseMessageFactory = serviceResponseMessageFactory;
    return this;
  }

  public MessageFactory getServiceResponseMessageFactory() {
    return serviceResponseMessageFactory;
  }

  /**
   * @param topicDescriptionFactory
   *          the {@link TopicDescriptionFactory} for the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setTopicDescriptionFactory(
      TopicDescriptionFactory topicDescriptionFactory) {
    this.topicDescriptionFactory = topicDescriptionFactory;
    return this;
  }

  public TopicDescriptionFactory getTopicDescriptionFactory() {
    return topicDescriptionFactory;
  }

  /**
   * @param serviceDescriptionFactory
   *          the {@link ServiceDescriptionFactory} for the {@link Node}
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setServiceDescriptionFactory(
      ServiceDescriptionFactory serviceDescriptionFactory) {
    this.serviceDescriptionFactory = serviceDescriptionFactory;
    return this;
  }

  public ServiceDescriptionFactory getServiceDescriptionFactory() {
    return serviceDescriptionFactory;
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/TCPROS">TCPROS documentation</a>
   * 
   * @return the {@link BindAddress} for the {@link Node}'s TCPROS server
   */
  public BindAddress getTcpRosBindAddress() {
    return tcpRosBindAddress;
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/TCPROS">TCPROS documentation</a>
   * 
   * @param tcpRosBindAddress
   *          the {@link BindAddress} for the {@link Node}'s TCPROS server
   */
  public NodeConfiguration setTcpRosBindAddress(BindAddress tcpRosBindAddress) {
    this.tcpRosBindAddress = tcpRosBindAddress;
    return this;
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/TCPROS">TCPROS documentation</a>
   * 
   * @return the {@link AdvertiseAddressFactory} for the {@link Node}'s TCPROS
   *         server
   */
  public AdvertiseAddressFactory getTcpRosAdvertiseAddressFactory() {
    return tcpRosAdvertiseAddressFactory;
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/TCPROS">TCPROS documentation</a>
   * 
   * @param tcpRosAdvertiseAddressFactory
   *          the {@link AdvertiseAddressFactory} for the {@link Node}'s TCPROS
   *          server
   * @return this {@link NodeConfiguration}
   */
  public NodeConfiguration setTcpRosAdvertiseAddressFactory(
      AdvertiseAddressFactory tcpRosAdvertiseAddressFactory) {
    this.tcpRosAdvertiseAddressFactory = tcpRosAdvertiseAddressFactory;
    return this;
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/TCPROS">TCPROS documentation</a>
   * 
   * @return the {@link AdvertiseAddress} for the {@link Node}'s TCPROS server
   */
  public AdvertiseAddress getTcpRosAdvertiseAddress() {
    return tcpRosAdvertiseAddressFactory.newDefault();
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/Technical%20Overview#Node">Node
   *      documentation</a>
   * 
   * @return the {@link BindAddress} for the {@link Node}'s XML-RPC server
   */
  public BindAddress getXmlRpcBindAddress() {
    return xmlRpcBindAddress;
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/Technical%20Overview#Node">Node
   *      documentation</a>
   * 
   * @param xmlRpcBindAddress
   *          the {@link BindAddress} for the {@link Node}'s XML-RPC server
   */
  public NodeConfiguration setXmlRpcBindAddress(BindAddress xmlRpcBindAddress) {
    this.xmlRpcBindAddress = xmlRpcBindAddress;
    return this;
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/Technical%20Overview#Node">Node
   *      documentation</a>
   * 
   * @return the {@link AdvertiseAddress} for the {@link Node}'s XML-RPC server
   */
  public AdvertiseAddress getXmlRpcAdvertiseAddress() {
    return xmlRpcAdvertiseAddressFactory.newDefault();
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/Technical%20Overview#Node">Node
   *      documentation</a>
   * 
   * @return the {@link AdvertiseAddressFactory} for the {@link Node}'s XML-RPC
   *         server
   */
  public AdvertiseAddressFactory getXmlRpcAdvertiseAddressFactory() {
    return xmlRpcAdvertiseAddressFactory;
  }

  /**
   * @see <a href="http://www.ros.org/wiki/ROS/Technical%20Overview#Node">Node
   *      documentation</a>
   * 
   * @param xmlRpcAdvertiseAddressFactory
   *          the {@link AdvertiseAddressFactory} for the {@link Node}'s XML-RPC
   *          server
   */
  public NodeConfiguration setXmlRpcAdvertiseAddressFactory(
      AdvertiseAddressFactory xmlRpcAdvertiseAddressFactory) {
    this.xmlRpcAdvertiseAddressFactory = xmlRpcAdvertiseAddressFactory;
    return this;
  }

  /**
   * @return the configured {@link TimeProvider}
   */
  public TimeProvider getTimeProvider() {
    return timeProvider;
  }

  /**
   * Sets the {@link TimeProvider} that {@link Node}s will use. By default, the
   * {@link WallTimeProvider} is used.
   * 
   * @param timeProvider
   *          the {@link TimeProvider} that {@link Node}s will use
   */
  public NodeConfiguration setTimeProvider(TimeProvider timeProvider) {
    this.timeProvider = timeProvider;
    return this;
  }
}
