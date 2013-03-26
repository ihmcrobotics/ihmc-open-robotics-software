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

package org.ros.internal.node;

import com.google.common.annotations.VisibleForTesting;

import org.apache.commons.logging.Log;
import org.ros.Parameters;
import org.ros.concurrent.CancellableLoop;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.message.service.ServiceDescription;
import org.ros.internal.message.topic.TopicDescription;
import org.ros.internal.node.client.MasterClient;
import org.ros.internal.node.client.Registrar;
import org.ros.internal.node.parameter.DefaultParameterTree;
import org.ros.internal.node.parameter.ParameterManager;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.StatusCode;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.server.SlaveServer;
import org.ros.internal.node.service.ServiceDeclaration;
import org.ros.internal.node.service.ServiceFactory;
import org.ros.internal.node.service.ServiceIdentifier;
import org.ros.internal.node.service.ServiceManager;
import org.ros.internal.node.topic.PublisherFactory;
import org.ros.internal.node.topic.SubscriberFactory;
import org.ros.internal.node.topic.TopicDeclaration;
import org.ros.internal.node.topic.TopicParticipantManager;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializationFactory;
import org.ros.message.MessageSerializer;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.namespace.NodeNameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.DefaultPublisherListener;
import org.ros.node.topic.DefaultSubscriberListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.time.ClockTopicTimeProvider;
import org.ros.time.TimeProvider;

import java.net.InetSocketAddress;
import java.net.URI;
import java.util.Collection;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * The default implementation of a {@link Node}.
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class DefaultNode implements ConnectedNode {

  private static final boolean DEBUG = false;

  // TODO(damonkohler): Move to NodeConfiguration.
  /**
   * The maximum delay before shutdown will begin even if all
   * {@link NodeListener}s have not yet returned from their
   * {@link NodeListener#onShutdown(Node)} callback.
   */
  private static final int MAX_SHUTDOWN_DELAY_DURATION = 5;
  private static final TimeUnit MAX_SHUTDOWN_DELAY_UNITS = TimeUnit.SECONDS;

  private final NodeConfiguration nodeConfiguration;
  private final ListenerGroup<NodeListener> nodeListeners;
  private final ScheduledExecutorService scheduledExecutorService;
  private final URI masterUri;
  private final MasterClient masterClient;
  private final TopicParticipantManager topicParticipantManager;
  private final ServiceManager serviceManager;
  private final ParameterManager parameterManager;
  private final GraphName nodeName;
  private final NodeNameResolver resolver;
  private final SlaveServer slaveServer;
  private final ParameterTree parameterTree;
  private final PublisherFactory publisherFactory;
  private final SubscriberFactory subscriberFactory;
  private final ServiceFactory serviceFactory;
  private final Registrar registrar;

  private RosoutLogger log;
  private TimeProvider timeProvider;

  /**
   * {@link DefaultNode}s should only be constructed using the
   * {@link DefaultNodeFactory}.
   * 
   * @param nodeConfiguration
   *          the {@link NodeConfiguration} for this {@link Node}
   * @param nodeListeners
   *          a {@link Collection} of {@link NodeListener}s that will be added
   *          to this {@link Node} before it starts
   */
  public DefaultNode(NodeConfiguration nodeConfiguration, Collection<NodeListener> nodeListeners,
      ScheduledExecutorService scheduledExecutorService) {
    this.nodeConfiguration = NodeConfiguration.copyOf(nodeConfiguration);
    this.nodeListeners = new ListenerGroup<NodeListener>(scheduledExecutorService);
    this.nodeListeners.addAll(nodeListeners);
    this.scheduledExecutorService = scheduledExecutorService;
    masterUri = nodeConfiguration.getMasterUri();
    masterClient = new MasterClient(masterUri);
    topicParticipantManager = new TopicParticipantManager();
    serviceManager = new ServiceManager();
    parameterManager = new ParameterManager(scheduledExecutorService);

    GraphName basename = nodeConfiguration.getNodeName();
    NameResolver parentResolver = nodeConfiguration.getParentResolver();
    nodeName = parentResolver.getNamespace().join(basename);
    resolver = new NodeNameResolver(nodeName, parentResolver);
    slaveServer =
        new SlaveServer(nodeName, nodeConfiguration.getTcpRosBindAddress(),
            nodeConfiguration.getTcpRosAdvertiseAddress(),
            nodeConfiguration.getXmlRpcBindAddress(),
            nodeConfiguration.getXmlRpcAdvertiseAddress(), masterClient, topicParticipantManager,
            serviceManager, parameterManager, scheduledExecutorService);
    slaveServer.start();

    NodeIdentifier nodeIdentifier = slaveServer.toNodeIdentifier();

    parameterTree =
        DefaultParameterTree.newFromNodeIdentifier(nodeIdentifier, masterClient.getRemoteUri(),
            resolver, parameterManager);

    publisherFactory =
        new PublisherFactory(nodeIdentifier, topicParticipantManager,
            nodeConfiguration.getTopicMessageFactory(), scheduledExecutorService);
    subscriberFactory =
        new SubscriberFactory(nodeIdentifier, topicParticipantManager, scheduledExecutorService);
    serviceFactory =
        new ServiceFactory(nodeName, slaveServer, serviceManager, scheduledExecutorService);

    registrar = new Registrar(masterClient, scheduledExecutorService);
    topicParticipantManager.setListener(registrar);
    serviceManager.setListener(registrar);

    scheduledExecutorService.execute(new Runnable() {
      @Override
      public void run() {
        start();
      }
    });
  }

  private void start() {
    // The Registrar must be started first so that master registration is
    // possible during startup.
    registrar.start(slaveServer.toNodeIdentifier());

    // During startup, we wait for 1) the RosoutLogger and 2) the TimeProvider.
    final CountDownLatch latch = new CountDownLatch(2);

    log = new RosoutLogger(this);
    log.getPublisher().addListener(new DefaultPublisherListener<rosgraph_msgs.Log>() {
      @Override
      public void onMasterRegistrationSuccess(Publisher<rosgraph_msgs.Log> registrant) {
        latch.countDown();
      }
    });

    boolean useSimTime = false;
    try {
      useSimTime =
          parameterTree.has(Parameters.USE_SIM_TIME)
              && parameterTree.getBoolean(Parameters.USE_SIM_TIME);
    } catch (Exception e) {
      signalOnError(e);
    }
    if (useSimTime) {
      ClockTopicTimeProvider clockTopicTimeProvider = new ClockTopicTimeProvider(this);
      clockTopicTimeProvider.getSubscriber().addSubscriberListener(
          new DefaultSubscriberListener<rosgraph_msgs.Clock>() {
            @Override
            public void onMasterRegistrationSuccess(Subscriber<rosgraph_msgs.Clock> subscriber) {
              latch.countDown();
            }
          });
      timeProvider = clockTopicTimeProvider;
    } else {
      timeProvider = nodeConfiguration.getTimeProvider();
      latch.countDown();
    }

    try {
      latch.await();
    } catch (InterruptedException e) {
      signalOnError(e);
      shutdown();
      return;
    }

    signalOnStart();
  }

  @VisibleForTesting
  Registrar getRegistrar() {
    return registrar;
  }

  private <T> org.ros.message.MessageSerializer<T> newMessageSerializer(String messageType) {
    return nodeConfiguration.getMessageSerializationFactory().newMessageSerializer(messageType);
  }

  @SuppressWarnings("unchecked")
  private <T> MessageDeserializer<T> newMessageDeserializer(String messageType) {
    return (MessageDeserializer<T>) nodeConfiguration.getMessageSerializationFactory()
        .newMessageDeserializer(messageType);
  }

  @SuppressWarnings("unchecked")
  private <T> MessageSerializer<T> newServiceResponseSerializer(String serviceType) {
    return (MessageSerializer<T>) nodeConfiguration.getMessageSerializationFactory()
        .newServiceResponseSerializer(serviceType);
  }

  @SuppressWarnings("unchecked")
  private <T> MessageDeserializer<T> newServiceResponseDeserializer(String serviceType) {
    return (MessageDeserializer<T>) nodeConfiguration.getMessageSerializationFactory()
        .newServiceResponseDeserializer(serviceType);
  }

  @SuppressWarnings("unchecked")
  private <T> MessageSerializer<T> newServiceRequestSerializer(String serviceType) {
    return (MessageSerializer<T>) nodeConfiguration.getMessageSerializationFactory()
        .newServiceRequestSerializer(serviceType);
  }

  @SuppressWarnings("unchecked")
  private <T> MessageDeserializer<T> newServiceRequestDeserializer(String serviceType) {
    return (MessageDeserializer<T>) nodeConfiguration.getMessageSerializationFactory()
        .newServiceRequestDeserializer(serviceType);
  }

  @Override
  public <T> Publisher<T> newPublisher(GraphName topicName, String messageType) {
    GraphName resolvedTopicName = resolveName(topicName);
    TopicDescription topicDescription =
        nodeConfiguration.getTopicDescriptionFactory().newFromType(messageType);
    TopicDeclaration topicDeclaration =
        TopicDeclaration.newFromTopicName(resolvedTopicName, topicDescription);
    org.ros.message.MessageSerializer<T> serializer = newMessageSerializer(messageType);
    return publisherFactory.newOrExisting(topicDeclaration, serializer);
  }

  @Override
  public <T> Publisher<T> newPublisher(String topicName, String messageType) {
    return newPublisher(GraphName.of(topicName), messageType);
  }

  @Override
  public <T> Subscriber<T> newSubscriber(GraphName topicName, String messageType) {
    GraphName resolvedTopicName = resolveName(topicName);
    TopicDescription topicDescription =
        nodeConfiguration.getTopicDescriptionFactory().newFromType(messageType);
    TopicDeclaration topicDeclaration =
        TopicDeclaration.newFromTopicName(resolvedTopicName, topicDescription);
    MessageDeserializer<T> deserializer = newMessageDeserializer(messageType);
    Subscriber<T> subscriber = subscriberFactory.newOrExisting(topicDeclaration, deserializer);
    return subscriber;
  }

  @Override
  public <T> Subscriber<T> newSubscriber(String topicName, String messageType) {
    return newSubscriber(GraphName.of(topicName), messageType);
  }

  @Override
  public <T, S> ServiceServer<T, S> newServiceServer(GraphName serviceName, String serviceType,
      ServiceResponseBuilder<T, S> responseBuilder) {
    GraphName resolvedServiceName = resolveName(serviceName);
    // TODO(damonkohler): It's rather non-obvious that the URI will be
    // created later on the fly.
    ServiceIdentifier identifier = new ServiceIdentifier(resolvedServiceName, null);
    ServiceDescription serviceDescription =
        nodeConfiguration.getServiceDescriptionFactory().newFromType(serviceType);
    ServiceDeclaration definition = new ServiceDeclaration(identifier, serviceDescription);
    MessageDeserializer<T> requestDeserializer = newServiceRequestDeserializer(serviceType);
    MessageSerializer<S> responseSerializer = newServiceResponseSerializer(serviceType);
    return serviceFactory.newServer(definition, responseBuilder, requestDeserializer,
        responseSerializer, nodeConfiguration.getServiceResponseMessageFactory());
  }

  @Override
  public <T, S> ServiceServer<T, S> newServiceServer(String serviceName, String serviceType,
      ServiceResponseBuilder<T, S> responseBuilder) {
    return newServiceServer(GraphName.of(serviceName), serviceType, responseBuilder);
  }

  @SuppressWarnings("unchecked")
  @Override
  public <T, S> ServiceServer<T, S> getServiceServer(GraphName serviceName) {
    return (ServiceServer<T, S>) serviceManager.getServer(serviceName);
  }

  @Override
  public <T, S> ServiceServer<T, S> getServiceServer(String serviceName) {
    return getServiceServer(GraphName.of(serviceName));
  }

  @Override
  public URI lookupServiceUri(GraphName serviceName) {
    Response<URI> response =
        masterClient.lookupService(slaveServer.toNodeIdentifier().getName(),
            resolveName(serviceName).toString());
    if (response.getStatusCode() == StatusCode.SUCCESS) {
      return response.getResult();
    } else {
      return null;
    }
  }

  @Override
  public URI lookupServiceUri(String serviceName) {
    return lookupServiceUri(GraphName.of(serviceName));
  }

  @Override
  public <T, S> ServiceClient<T, S> newServiceClient(GraphName serviceName, String serviceType)
      throws ServiceNotFoundException {
    GraphName resolvedServiceName = resolveName(serviceName);
    URI uri = lookupServiceUri(resolvedServiceName);
    if (uri == null) {
      throw new ServiceNotFoundException("No such service " + resolvedServiceName + " of type "
          + serviceType);
    }
    ServiceDescription serviceDescription =
        nodeConfiguration.getServiceDescriptionFactory().newFromType(serviceType);
    ServiceIdentifier serviceIdentifier = new ServiceIdentifier(resolvedServiceName, uri);
    ServiceDeclaration definition = new ServiceDeclaration(serviceIdentifier, serviceDescription);
    MessageSerializer<T> requestSerializer = newServiceRequestSerializer(serviceType);
    MessageDeserializer<S> responseDeserializer = newServiceResponseDeserializer(serviceType);
    return serviceFactory.newClient(definition, requestSerializer, responseDeserializer,
        nodeConfiguration.getServiceRequestMessageFactory());
  }

  @Override
  public <T, S> ServiceClient<T, S> newServiceClient(String serviceName, String serviceType)
      throws ServiceNotFoundException {
    return newServiceClient(GraphName.of(serviceName), serviceType);
  }

  @Override
  public Time getCurrentTime() {
    return timeProvider.getCurrentTime();
  }

  @Override
  public GraphName getName() {
    return nodeName;
  }

  @Override
  public Log getLog() {
    return log;
  }

  @Override
  public GraphName resolveName(GraphName name) {
    return resolver.resolve(name);
  }

  @Override
  public GraphName resolveName(String name) {
    return resolver.resolve(GraphName.of(name));
  }

  @Override
  public void shutdown() {
    signalOnShutdown();
    // NOTE(damonkohler): We don't want to raise potentially spurious
    // exceptions during shutdown that would interrupt the process. This is
    // simply best effort cleanup.
    for (Publisher<?> publisher : topicParticipantManager.getPublishers()) {
      publisher.shutdown();
    }
    for (Subscriber<?> subscriber : topicParticipantManager.getSubscribers()) {
      subscriber.shutdown();
    }
    for (ServiceServer<?, ?> serviceServer : serviceManager.getServers()) {
      try {
        Response<Integer> response =
            masterClient.unregisterService(slaveServer.toNodeIdentifier(), serviceServer);
        if (DEBUG) {
          if (response.getResult() == 0) {
            System.err.println("Failed to unregister service: " + serviceServer.getName());
          }
        }
      } catch (XmlRpcTimeoutException e) {
        log.error(e);
      } catch (RemoteException e) {
        log.error(e);
      }
    }
    for (ServiceClient<?, ?> serviceClient : serviceManager.getClients()) {
      serviceClient.shutdown();
    }
    registrar.shutdown();
    slaveServer.shutdown();
    signalOnShutdownComplete();
  }

  @Override
  public URI getMasterUri() {
    return masterUri;
  }

  @Override
  public NodeNameResolver getResolver() {
    return resolver;
  }

  @Override
  public ParameterTree getParameterTree() {
    return parameterTree;
  }

  @Override
  public URI getUri() {
    return slaveServer.getUri();
  }

  @Override
  public MessageSerializationFactory getMessageSerializationFactory() {
    return nodeConfiguration.getMessageSerializationFactory();
  }

  @Override
  public MessageFactory getTopicMessageFactory() {
    return nodeConfiguration.getTopicMessageFactory();
  }

  @Override
  public MessageFactory getServiceRequestMessageFactory() {
    return nodeConfiguration.getServiceRequestMessageFactory();
  }

  @Override
  public MessageFactory getServiceResponseMessageFactory() {
    return nodeConfiguration.getServiceResponseMessageFactory();
  }

  @Override
  public void addListener(NodeListener listener) {
    nodeListeners.add(listener);
  }

  /**
   * SignalRunnable all {@link NodeListener}s that the {@link Node} has
   * experienced an error.
   * <p>
   * Each listener is called in a separate thread.
   */
  private void signalOnError(final Throwable throwable) {
    final Node node = this;
    nodeListeners.signal(new SignalRunnable<NodeListener>() {
      @Override
      public void run(NodeListener listener) {
        listener.onError(node, throwable);
      }
    });
  }

  /**
   * SignalRunnable all {@link NodeListener}s that the {@link Node} has started.
   * <p>
   * Each listener is called in a separate thread.
   */
  private void signalOnStart() {
    final ConnectedNode connectedNode = this;
    nodeListeners.signal(new SignalRunnable<NodeListener>() {
      @Override
      public void run(NodeListener listener) {
        listener.onStart(connectedNode);
      }
    });
  }

  /**
   * SignalRunnable all {@link NodeListener}s that the {@link Node} has started
   * shutting down.
   * <p>
   * Each listener is called in a separate thread.
   */
  private void signalOnShutdown() {
    final Node node = this;
    try {
      nodeListeners.signal(new SignalRunnable<NodeListener>() {
        @Override
        public void run(NodeListener listener) {
          listener.onShutdown(node);
        }
      }, MAX_SHUTDOWN_DELAY_DURATION, MAX_SHUTDOWN_DELAY_UNITS);
    } catch (InterruptedException e) {
      // Ignored since we do not guarantee that all listeners will finish
      // before
      // shutdown begins.
    }
  }

  /**
   * SignalRunnable all {@link NodeListener}s that the {@link Node} has shut
   * down.
   * <p>
   * Each listener is called in a separate thread.
   */
  private void signalOnShutdownComplete() {
    final Node node = this;
    nodeListeners.signal(new SignalRunnable<NodeListener>() {
      @Override
      public void run(NodeListener listener) {
        try {
          listener.onShutdownComplete(node);
        } catch (Throwable e) {
          System.out.println(listener);
        }
      }
    });
  }

  @VisibleForTesting
  InetSocketAddress getAddress() {
    return slaveServer.getAddress();
  }

  @Override
  public ScheduledExecutorService getScheduledExecutorService() {
    return scheduledExecutorService;
  }

  @Override
  public void executeCancellableLoop(final CancellableLoop cancellableLoop) {
    scheduledExecutorService.execute(cancellableLoop);
    addListener(new NodeListener() {
      @Override
      public void onStart(ConnectedNode connectedNode) {
      }

      @Override
      public void onShutdown(Node node) {
        cancellableLoop.cancel();
      }

      @Override
      public void onShutdownComplete(Node node) {
      }

      @Override
      public void onError(Node node, Throwable throwable) {
        cancellableLoop.cancel();
      }
    });
  }
}
