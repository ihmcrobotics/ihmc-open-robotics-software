//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.ros.node;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
//import org.ros.address.AdvertiseAddress;
//import org.ros.address.AdvertiseAddressFactory;
//import org.ros.address.BindAddress;
//import org.ros.address.PrivateAdvertiseAddressFactory;
//import org.ros.address.PublicAdvertiseAddressFactory;
//import org.ros.exception.RosRuntimeException;
//import org.ros.internal.message.DefaultMessageFactory;
//import org.ros.internal.message.DefaultMessageSerializationFactory;
//import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
//import org.ros.internal.message.service.ServiceDescriptionFactory;
//import org.ros.internal.message.service.ServiceRequestMessageFactory;
//import org.ros.internal.message.service.ServiceResponseMessageFactory;
//import org.ros.internal.message.topic.TopicDescriptionFactory;
//import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
//import org.ros.message.MessageSerializationFactory;
import org.ros.namespace.GraphName;
//import org.ros.namespace.NameResolver;
import org.ros.time.TimeProvider;
import us.ihmc.log.LogTools;
//import org.ros.time.WallTimeProvider;

public class NodeConfiguration {
   public static URI DEFAULT_MASTER_URI;
//   private NameResolver parentResolver;
   private URI masterUri;
   private File rosRoot;
   private List<File> rosPackagePath;
   private GraphName nodeName;
//   private TopicDescriptionFactory topicDescriptionFactory;
//   private MessageFactory topicMessageFactory;
//   private ServiceDescriptionFactory serviceDescriptionFactory;
//   private MessageFactory serviceRequestMessageFactory;
//   private MessageFactory serviceResponseMessageFactory;
//   private MessageSerializationFactory messageSerializationFactory;
//   private BindAddress tcpRosBindAddress;
//   private AdvertiseAddressFactory tcpRosAdvertiseAddressFactory;
//   private BindAddress xmlRpcBindAddress;
//   private AdvertiseAddressFactory xmlRpcAdvertiseAddressFactory;
   private ScheduledExecutorService scheduledExecutorService;
//   private TimeProvider timeProvider;

   public static NodeConfiguration copyOf(NodeConfiguration nodeConfiguration) {
      NodeConfiguration copy = new NodeConfiguration();
//      copy.parentResolver = nodeConfiguration.parentResolver;
      copy.masterUri = nodeConfiguration.masterUri;
      copy.rosRoot = nodeConfiguration.rosRoot;
      copy.rosPackagePath = nodeConfiguration.rosPackagePath;
      copy.nodeName = nodeConfiguration.nodeName;
//      copy.topicDescriptionFactory = nodeConfiguration.topicDescriptionFactory;
//      copy.topicMessageFactory = nodeConfiguration.topicMessageFactory;
//      copy.serviceDescriptionFactory = nodeConfiguration.serviceDescriptionFactory;
//      copy.serviceRequestMessageFactory = nodeConfiguration.serviceRequestMessageFactory;
//      copy.serviceResponseMessageFactory = nodeConfiguration.serviceResponseMessageFactory;
//      copy.messageSerializationFactory = nodeConfiguration.messageSerializationFactory;
//      copy.tcpRosBindAddress = nodeConfiguration.tcpRosBindAddress;
//      copy.tcpRosAdvertiseAddressFactory = nodeConfiguration.tcpRosAdvertiseAddressFactory;
//      copy.xmlRpcBindAddress = nodeConfiguration.xmlRpcBindAddress;
//      copy.xmlRpcAdvertiseAddressFactory = nodeConfiguration.xmlRpcAdvertiseAddressFactory;
      copy.scheduledExecutorService = nodeConfiguration.scheduledExecutorService;
//      copy.timeProvider = nodeConfiguration.timeProvider;
      return copy;
   }

   public static NodeConfiguration newPublic(String host, URI masterUri) {
      NodeConfiguration configuration = new NodeConfiguration();
//      configuration.setXmlRpcBindAddress(BindAddress.newPublic());
//      configuration.setXmlRpcAdvertiseAddressFactory(new PublicAdvertiseAddressFactory(host));
//      configuration.setTcpRosBindAddress(BindAddress.newPublic());
//      configuration.setTcpRosAdvertiseAddressFactory(new PublicAdvertiseAddressFactory(host));
//      configuration.setMasterUri(masterUri);
      return configuration;
   }

   public static NodeConfiguration newPublic(String host) {
      return newPublic(host, DEFAULT_MASTER_URI);
   }

   public static NodeConfiguration newPrivate(URI masterUri) {
      NodeConfiguration configuration = new NodeConfiguration();
//      configuration.setXmlRpcBindAddress(BindAddress.newPrivate());
//      configuration.setXmlRpcAdvertiseAddressFactory(new PrivateAdvertiseAddressFactory());
//      configuration.setTcpRosBindAddress(BindAddress.newPrivate());
//      configuration.setTcpRosAdvertiseAddressFactory(new PrivateAdvertiseAddressFactory());
//      configuration.setMasterUri(masterUri);
      return configuration;
   }

   public static NodeConfiguration newPrivate() {
      return newPrivate(DEFAULT_MASTER_URI);
   }

   private NodeConfiguration() {
//      MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
//      this.setTopicDescriptionFactory(new TopicDescriptionFactory(messageDefinitionProvider));
//      this.setTopicMessageFactory(new DefaultMessageFactory(messageDefinitionProvider));
//      this.setServiceDescriptionFactory(new ServiceDescriptionFactory(messageDefinitionProvider));
//      this.setServiceRequestMessageFactory(new ServiceRequestMessageFactory(messageDefinitionProvider));
//      this.setServiceResponseMessageFactory(new ServiceResponseMessageFactory(messageDefinitionProvider));
//      this.setMessageSerializationFactory(new DefaultMessageSerializationFactory(messageDefinitionProvider));
//      this.setParentResolver(NameResolver.newRoot());
//      this.setTimeProvider(new WallTimeProvider());
   }

//   public NameResolver getParentResolver() {
//      return this.parentResolver;
//   }
//
//   public NodeConfiguration setParentResolver(NameResolver resolver) {
//      this.parentResolver = resolver;
//      return this;
//   }

   public URI getMasterUri() {
      return this.masterUri;
   }

   public NodeConfiguration setMasterUri(URI masterUri) {
      this.masterUri = masterUri;
      return this;
   }

   public File getRosRoot() {
      return this.rosRoot;
   }

   public NodeConfiguration setRosRoot(File rosRoot) {
      this.rosRoot = rosRoot;
      return this;
   }

   public List<File> getRosPackagePath() {
      return this.rosPackagePath;
   }

   public NodeConfiguration setRosPackagePath(List<File> rosPackagePath) {
      this.rosPackagePath = rosPackagePath;
      return this;
   }

   public GraphName getNodeName() {
      return this.nodeName;
   }

   public NodeConfiguration setNodeName(GraphName nodeName) {
      this.nodeName = nodeName;
      return this;
   }

   public NodeConfiguration setNodeName(String nodeName) {
      return this.setNodeName(GraphName.of(nodeName));
   }

   public NodeConfiguration setDefaultNodeName(GraphName nodeName) {
      if (this.nodeName == null) {
         this.setNodeName(nodeName);
      }

      return this;
   }

   public NodeConfiguration setDefaultNodeName(String nodeName) {
      return this.setDefaultNodeName(GraphName.of(nodeName));
   }

//   public MessageSerializationFactory getMessageSerializationFactory() {
//      return this.messageSerializationFactory;
//   }
//
//   public NodeConfiguration setMessageSerializationFactory(MessageSerializationFactory messageSerializationFactory) {
//      this.messageSerializationFactory = messageSerializationFactory;
//      return this;
//   }
//
//   public NodeConfiguration setTopicMessageFactory(MessageFactory topicMessageFactory) {
//      this.topicMessageFactory = topicMessageFactory;
//      return this;
//   }

   public MessageFactory getTopicMessageFactory() {
      return null;
   }

//   public NodeConfiguration setServiceRequestMessageFactory(ServiceRequestMessageFactory serviceRequestMessageFactory) {
//      this.serviceRequestMessageFactory = serviceRequestMessageFactory;
//      return this;
//   }
//
//   public MessageFactory getServiceRequestMessageFactory() {
//      return this.serviceRequestMessageFactory;
//   }
//
//   public NodeConfiguration setServiceResponseMessageFactory(ServiceResponseMessageFactory serviceResponseMessageFactory) {
//      this.serviceResponseMessageFactory = serviceResponseMessageFactory;
//      return this;
//   }
//
//   public MessageFactory getServiceResponseMessageFactory() {
//      return this.serviceResponseMessageFactory;
//   }
//
//   public NodeConfiguration setTopicDescriptionFactory(TopicDescriptionFactory topicDescriptionFactory) {
//      this.topicDescriptionFactory = topicDescriptionFactory;
//      return this;
//   }
//
//   public TopicDescriptionFactory getTopicDescriptionFactory() {
//      return this.topicDescriptionFactory;
//   }
//
//   public NodeConfiguration setServiceDescriptionFactory(ServiceDescriptionFactory serviceDescriptionFactory) {
//      this.serviceDescriptionFactory = serviceDescriptionFactory;
//      return this;
//   }
//
//   public ServiceDescriptionFactory getServiceDescriptionFactory() {
//      return this.serviceDescriptionFactory;
//   }
//
//   public BindAddress getTcpRosBindAddress() {
//      return this.tcpRosBindAddress;
//   }

//   public NodeConfiguration setTcpRosBindAddress(BindAddress tcpRosBindAddress) {
//      this.tcpRosBindAddress = tcpRosBindAddress;
//      return this;
//   }
//
//   public AdvertiseAddressFactory getTcpRosAdvertiseAddressFactory() {
//      return this.tcpRosAdvertiseAddressFactory;
//   }
//
//   public NodeConfiguration setTcpRosAdvertiseAddressFactory(AdvertiseAddressFactory tcpRosAdvertiseAddressFactory) {
//      this.tcpRosAdvertiseAddressFactory = tcpRosAdvertiseAddressFactory;
//      return this;
//   }
//
//   public AdvertiseAddress getTcpRosAdvertiseAddress() {
//      return this.tcpRosAdvertiseAddressFactory.newDefault();
//   }
//
//   public BindAddress getXmlRpcBindAddress() {
//      return this.xmlRpcBindAddress;
//   }
//
//   public NodeConfiguration setXmlRpcBindAddress(BindAddress xmlRpcBindAddress) {
//      this.xmlRpcBindAddress = xmlRpcBindAddress;
//      return this;
//   }
//
//   public AdvertiseAddress getXmlRpcAdvertiseAddress() {
//      return this.xmlRpcAdvertiseAddressFactory.newDefault();
//   }
//
//   public AdvertiseAddressFactory getXmlRpcAdvertiseAddressFactory() {
//      return this.xmlRpcAdvertiseAddressFactory;
//   }
//
//   public NodeConfiguration setXmlRpcAdvertiseAddressFactory(AdvertiseAddressFactory xmlRpcAdvertiseAddressFactory) {
//      this.xmlRpcAdvertiseAddressFactory = xmlRpcAdvertiseAddressFactory;
//      return this;
//   }

   public TimeProvider getTimeProvider() {
      return null;
   }

   public NodeConfiguration setTimeProvider(TimeProvider timeProvider) {
//      this.timeProvider = timeProvider;
      return this;
   }

   static {
      try {
         DEFAULT_MASTER_URI = new URI("http://localhost:11311/");
      } catch (URISyntaxException var1) {
//         throw new RosRuntimeException(var1);
         LogTools.error("sjdasd");
      }
   }
}
