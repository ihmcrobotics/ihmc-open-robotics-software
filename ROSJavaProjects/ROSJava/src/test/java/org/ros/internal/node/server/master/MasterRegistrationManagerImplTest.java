/**
 * 
 */
package org.ros.internal.node.server.master;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.google.common.collect.Sets;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;

import java.net.URI;

/**
 * Tests for the {@link MasterRegistrationManagerImpl}.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class MasterRegistrationManagerImplTest {

  private MasterRegistrationManagerImpl masterRegistrationManager;
  private MasterRegistrationListener registrationListener;

  @Before
  public void setup() {
    registrationListener = mock(MasterRegistrationListener.class);
    masterRegistrationManager = new MasterRegistrationManagerImpl(registrationListener);
  }

  /**
   * Register only a publisher and no subscribers.
   * 
   * @throws Exception
   */
  @Test
  public void testRegisterOnlyPublisher() throws Exception {
    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri = new URI("http://localhost:12345");
    GraphName topicName = GraphName.of("/topic");
    String topicMessageType = "topic/Message";

    TopicRegistrationInfo topic =
        masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri, topicName,
            topicMessageType);

    assertTrue(topic.getSubscribers().isEmpty());

    assertEquals(topicMessageType, topic.getMessageType());

    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);
    assertTrue(node.hasRegistrations());
    assertTrue(node.getSubscribers().isEmpty());
    assertEquals(Sets.newHashSet(topic), node.getPublishers());

    // Make sure only publisher in the topic is the topic we got for the node
    // name
    assertEquals(Sets.newHashSet(node), topic.getPublishers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node);
  }

  /**
   * Register only a subscriber and no publishers.
   * 
   * @throws Exception
   */
  @Test
  public void testRegisterOnlySubscriber() throws Exception {
    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri = new URI("http://localhost:12345");
    GraphName topicName = GraphName.of("/topic");
    String topicMessageType = "topic/Message";

    TopicRegistrationInfo topic =
        masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName,
            topicMessageType);

    assertTrue(topic.getPublishers().isEmpty());

    assertEquals(topicMessageType, topic.getMessageType());

    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);
    assertTrue(node.hasRegistrations());
    assertTrue(node.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topic), node.getSubscribers());

    // Make sure only subscriber in the topic is the topic we got for the node
    // name
    assertEquals(Sets.newHashSet(node), topic.getSubscribers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node);
  }

  /**
   * Register only a subscriber and no publishers. TopicSystemState message type will be
   * the wildcard, there should be no topic type.
   * 
   * @throws Exception
   */
  @Test
  public void testOnlySubscriberWildcard() throws Exception {
    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri = new URI("http://localhost:12345");
    GraphName topicName = GraphName.of("/topic");

    TopicRegistrationInfo topic =
        masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName,
            Subscriber.TOPIC_MESSAGE_TYPE_WILDCARD);

    // The important result. All of the others are to make sure the wildcard
    // didn't
    // screw anything else up.
    assertNull(topic.getMessageType());

    assertTrue(topic.getPublishers().isEmpty());

    // Make sure subscriber was still registered.
    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);
    assertTrue(node.hasRegistrations());
    assertTrue(node.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topic), node.getSubscribers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node);
  }

  /**
   * Register two subscribers, each with their own node. Both subscribers should
   * appear. The topic message type should be the second subscriber's message
   * type.
   * 
   * @throws Exception
   */
  @Test
  public void testSubscriberSubscriber() throws Exception {
    GraphName topicName = GraphName.of("/topic");

    GraphName nodeName1 = GraphName.of("/node1");
    URI nodeSlaveUri1 = new URI("http://localhost:12345");
    String topicMessageType1 = "topic/Message1";

    TopicRegistrationInfo topic1 =
        masterRegistrationManager.registerSubscriber(nodeName1, nodeSlaveUri1, topicName,
            topicMessageType1);

    GraphName nodeName2 = GraphName.of("/node2");
    URI nodeSlaveUri2 = new URI("http://localhost:54321");
    String topicMessageType2 = "topic/Message2";

    TopicRegistrationInfo topic2 =
        masterRegistrationManager.registerSubscriber(nodeName2, nodeSlaveUri2, topicName,
            topicMessageType2);

    // This is the important test. The message type should be the second
    // registration's type.
    assertEquals(topicMessageType2, topic2.getMessageType());

    // Two topic descriptions should be the same
    assertTrue(topic1 == topic2);

    // Check everything else to make sure stayed the same.
    assertTrue(topic1.getPublishers().isEmpty());
    NodeRegistrationInfo node1 = masterRegistrationManager.getNodeRegistrationInfo(nodeName1);
    assertTrue(node1.hasRegistrations());
    assertTrue(node1.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topic1), node1.getSubscribers());

    assertTrue(topic2.getPublishers().isEmpty());
    NodeRegistrationInfo node2 = masterRegistrationManager.getNodeRegistrationInfo(nodeName2);
    assertTrue(node2.hasRegistrations());
    assertTrue(node2.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topic1), node2.getSubscribers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node1);
    verify(registrationListener, Mockito.never()).onNodeReplacement(node2);
  }

  /**
   * Register two subscribers and no publishers. The first subscriber's topic
   * will be real, the second will be the wildcard. The topic type should be the
   * first subscriber's.
   * 
   * @throws Exception
   */
  @Test
  public void testSubscriberSubscriberWildcard() throws Exception {
    GraphName topicName = GraphName.of("/topic");

    GraphName nodeName1 = GraphName.of("/node1");
    URI nodeSlaveUri1 = new URI("http://localhost:12345");
    String topicMessageType1 = "msgs/Message";

    TopicRegistrationInfo topic1 =
        masterRegistrationManager.registerSubscriber(nodeName1, nodeSlaveUri1, topicName,
            topicMessageType1);

    GraphName nodeName2 = GraphName.of("/node2");
    URI nodeSlaveUri2 = new URI("http://localhost:54321");

    TopicRegistrationInfo topic2 =
        masterRegistrationManager.registerSubscriber(nodeName2, nodeSlaveUri2, topicName,
            Subscriber.TOPIC_MESSAGE_TYPE_WILDCARD);

    assertTrue(topic1.getPublishers().isEmpty());

    // Two topic information objects should be the same
    assertTrue(topic1 == topic2);

    assertEquals(topicMessageType1, topic1.getMessageType());

    // Make sure subscribers were still registered for both nodes.
    NodeRegistrationInfo node1 = masterRegistrationManager.getNodeRegistrationInfo(nodeName1);
    assertTrue(node1.hasRegistrations());
    assertTrue(node1.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topic1), node1.getSubscribers());

    NodeRegistrationInfo node2 = masterRegistrationManager.getNodeRegistrationInfo(nodeName2);
    assertTrue(node2.hasRegistrations());
    assertTrue(node2.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topic1), node2.getSubscribers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node1);
    verify(registrationListener, Mockito.never()).onNodeReplacement(node2);
  }

  /**
   * Register one subscriber and one publisher, in that order. The subscriber
   * topic message type will be overwritten by the publishers.
   * 
   * @throws Exception
   */
  @Test
  public void testSubscriberPublisher() throws Exception {
    GraphName topicName = GraphName.of("/topic");

    GraphName nodeNameSubscriber = GraphName.of("/node1");
    URI nodeSlaveUriSubscriber = new URI("http://localhost:12345");
    String topicMessageTypeSubscriber = "msgs/Message1";

    TopicRegistrationInfo topicSubscriber =
        masterRegistrationManager.registerSubscriber(nodeNameSubscriber, nodeSlaveUriSubscriber,
            topicName, topicMessageTypeSubscriber);

    GraphName nodeNamePublisher = GraphName.of("/node2");
    URI nodeSlaveUriPublisher = new URI("http://localhost:54321");
    String topicMessageTypePublisher = "msgs/Message2";

    TopicRegistrationInfo topicPublisher =
        masterRegistrationManager.registerPublisher(nodeNamePublisher, nodeSlaveUriPublisher,
            topicName, topicMessageTypePublisher);

    // Should be only one topic info object since the same topic type
    assertTrue(topicSubscriber == topicPublisher);

    // Publisher will beat out subscriber message type
    assertEquals(topicMessageTypePublisher, topicSubscriber.getMessageType());

    // Make sure node info objects have everything
    NodeRegistrationInfo node1 =
        masterRegistrationManager.getNodeRegistrationInfo(nodeNameSubscriber);
    assertTrue(node1.hasRegistrations());
    assertTrue(node1.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topicSubscriber), node1.getSubscribers());

    NodeRegistrationInfo node2 =
        masterRegistrationManager.getNodeRegistrationInfo(nodeNamePublisher);
    assertTrue(node2.hasRegistrations());
    assertTrue(node2.getSubscribers().isEmpty());
    assertEquals(Sets.newHashSet(topicSubscriber), node2.getPublishers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node1);
    verify(registrationListener, Mockito.never()).onNodeReplacement(node2);
  }

  /**
   * Register one publisher and one subscriber, in that order. The subscriber
   * topic message type will not be overwritten by the subscriber.
   * 
   * @throws Exception
   */
  @Test
  public void testPublisherSubscriber() throws Exception {
    GraphName topicName = GraphName.of("/topic");

    GraphName nodeNamePublisher = GraphName.of("/node2");
    URI nodeSlaveUriPublisher = new URI("http://localhost:54321");
    String topicMessageTypePublisher = "msgs/Message2";

    TopicRegistrationInfo topicPublisher =
        masterRegistrationManager.registerPublisher(nodeNamePublisher, nodeSlaveUriPublisher,
            topicName, topicMessageTypePublisher);

    GraphName nodeNameSubscriber = GraphName.of("/node1");
    URI nodeSlaveUriSubscriber = new URI("http://localhost:12345");
    String topicMessageTypeSubscriber = "msgs/Message1";

    TopicRegistrationInfo topicSubscriber =
        masterRegistrationManager.registerSubscriber(nodeNameSubscriber, nodeSlaveUriSubscriber,
            topicName, topicMessageTypeSubscriber);

    // Should be only one topic info object since the same topic type
    assertTrue(topicSubscriber == topicPublisher);

    // Publisher message type stayed
    assertEquals(topicMessageTypePublisher, topicSubscriber.getMessageType());

    // Make sure node info objects have everything
    NodeRegistrationInfo node1 =
        masterRegistrationManager.getNodeRegistrationInfo(nodeNameSubscriber);
    assertTrue(node1.hasRegistrations());
    assertTrue(node1.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topicSubscriber), node1.getSubscribers());

    NodeRegistrationInfo node2 =
        masterRegistrationManager.getNodeRegistrationInfo(nodeNamePublisher);
    assertTrue(node2.hasRegistrations());
    assertTrue(node2.getSubscribers().isEmpty());
    assertEquals(Sets.newHashSet(topicSubscriber), node2.getPublishers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node1);
    verify(registrationListener, Mockito.never()).onNodeReplacement(node2);
  }

  /**
   * Register two publishers. The topic message type will be overwritten by the
   * second publisher.
   * 
   * @throws Exception
   */
  @Test
  public void testPublisherPublisher() throws Exception {
    GraphName topicName = GraphName.of("/topic");

    GraphName nodeNamePublisher1 = GraphName.of("/node1");
    URI nodeSlaveUriPublisher1 = new URI("http://localhost:54321");
    String topicMessageTypePublisher1 = "msgs/Message1";

    TopicRegistrationInfo topicPublisher1 =
        masterRegistrationManager.registerPublisher(nodeNamePublisher1, nodeSlaveUriPublisher1,
            topicName, topicMessageTypePublisher1);

    GraphName nodeNamePublisher2 = GraphName.of("/node2");
    URI nodeSlaveUriPublisher2 = new URI("http://localhost:12345");
    String topicMessageTypePublisher2 = "msgs/Message2";

    TopicRegistrationInfo topicPublisher2 =
        masterRegistrationManager.registerPublisher(nodeNamePublisher2, nodeSlaveUriPublisher2,
            topicName, topicMessageTypePublisher2);

    // Should be only one topic info object since the same topic type
    assertTrue(topicPublisher2 == topicPublisher1);

    // Second message type better have won.
    assertEquals(topicMessageTypePublisher2, topicPublisher2.getMessageType());

    // Make sure node info objects have everything
    NodeRegistrationInfo node1 =
        masterRegistrationManager.getNodeRegistrationInfo(nodeNamePublisher2);
    assertTrue(node1.hasRegistrations());
    assertTrue(node1.getSubscribers().isEmpty());
    assertEquals(Sets.newHashSet(topicPublisher2), node1.getPublishers());

    NodeRegistrationInfo node2 =
        masterRegistrationManager.getNodeRegistrationInfo(nodeNamePublisher1);
    assertTrue(node2.hasRegistrations());
    assertTrue(node2.getSubscribers().isEmpty());
    assertEquals(Sets.newHashSet(topicPublisher2), node2.getPublishers());

    // Both publishers in topic
    assertEquals(Sets.newHashSet(node1, node2), topicPublisher1.getPublishers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node1);
    verify(registrationListener, Mockito.never()).onNodeReplacement(node2);
  }

  /**
   * Register two publishers, each with the same node name but different URIs.
   * Make sure the first one gets shut down.
   * 
   * @throws Exception
   */
  @Test
  public void testNodeNameRepeated() throws Exception {
    GraphName nodeName = GraphName.of("/node1");

    GraphName topicName1 = GraphName.of("/topic1");
    URI nodeSlaveUri1 = new URI("http://localhost:54321");
    String topicMessageType1 = "msgs/Message1";

    TopicRegistrationInfo topic1 =
        masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri1, topicName1,
            topicMessageType1);

    NodeRegistrationInfo node1 = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    URI nodeSlaveUri2 = new URI("http://localhost:12345");
    GraphName topicName2 = GraphName.of("/topic2");
    String topicMessageType2 = "msgs/Message2";

    TopicRegistrationInfo topic2 =
        masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri2, topicName2,
            topicMessageType2);

    NodeRegistrationInfo node2 = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    // Make sure node1 signaled a replacement
    verify(registrationListener, Mockito.times(1)).onNodeReplacement(node1);

    // Should be two topics.
    assertFalse(topic2 == topic1);

    // Definitely second registration's message type better be the current
    // message type.
    assertEquals(topicMessageType2, topic2.getMessageType());

    // TopicSystemState one should have no publishers at all
    assertTrue(topic1.getPublishers().isEmpty());

    // Node 2 is fully there.
    assertTrue(node2.hasRegistrations());
    assertTrue(node2.getSubscribers().isEmpty());
    assertEquals(Sets.newHashSet(topic2), node2.getPublishers());
  }

  /**
   * Register a single publisher and then delete it.
   * 
   * @throws Exception
   */
  @Test
  public void testSinglePublisherRemoved() throws Exception {
    GraphName nodeName = GraphName.of("/node1");
    GraphName topicName = GraphName.of("/topic1");
    URI nodeSlaveUri = new URI("http://localhost:54321");
    String topicMessageType = "msgs/Message1";

    TopicRegistrationInfo topic =
        masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri, topicName,
            topicMessageType);

    masterRegistrationManager.unregisterPublisher(nodeName, topicName);

    // TopicSystemState should have no publishers at all
    assertTrue(topic.getPublishers().isEmpty());

    // Node should have been deleted.
    assertNull(masterRegistrationManager.getNodeRegistrationInfo(nodeName));
  }

  /**
   * Register two publishers on the same node and then delete one of them.
   * 
   * @throws Exception
   */
  @Test
  public void testTwoPublishersOneRemoved() throws Exception {
    GraphName nodeName = GraphName.of("/node1");
    URI nodeSlaveUri = new URI("http://localhost:54321");

    GraphName topicName1 = GraphName.of("/topic1");
    String topicMessageType1 = "msgs/Message1";

    TopicRegistrationInfo topic1 =
        masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri, topicName1,
            topicMessageType1);

    GraphName topicName2 = GraphName.of("/topic2");
    String topicMessageType2 = "msgs/Message2";

    TopicRegistrationInfo topic2 =
        masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri, topicName2,
            topicMessageType2);

    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    // Delete the first
    masterRegistrationManager.unregisterPublisher(nodeName, topicName1);

    // TopicSystemState 1 should have no publishers at all. TopicSystemState 2 will have 1 publisher,
    // and the node will show this published topic.
    assertTrue(topic1.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topic2), node.getPublishers());
    assertEquals(Sets.newHashSet(node), topic2.getPublishers());
  }

  /**
   * Register a single subscriber and then delete it.
   * 
   * @throws Exception
   */
  @Test
  public void testSingleSubscriberRemoved() throws Exception {
    GraphName nodeName = GraphName.of("/node1");
    GraphName topicName = GraphName.of("/topic1");
    URI nodeSlaveUri = new URI("http://localhost:54321");
    String topicMessageType = "msgs/Message1";

    TopicRegistrationInfo topic =
        masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName,
            topicMessageType);

    masterRegistrationManager.unregisterSubscriber(nodeName, topicName);

    // TopicSystemState one should have no subscribers at all
    assertTrue(topic.getSubscribers().isEmpty());

    // Node should have been deleted.
    assertNull(masterRegistrationManager.getNodeRegistrationInfo(nodeName));
  }

  /**
   * Register two subscribers on the same node and then delete one of them.
   * 
   * @throws Exception
   */
  @Test
  public void testTwoSubscribersOneRemoved() throws Exception {
    GraphName nodeName = GraphName.of("/node1");
    URI nodeSlaveUri = new URI("http://localhost:54321");

    GraphName topicName1 = GraphName.of("/topic1");
    String topicMessageType1 = "msgs/Message1";

    TopicRegistrationInfo topic1 =
        masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName1,
            topicMessageType1);

    GraphName topicName2 = GraphName.of("/topic2");
    String topicMessageType2 = "msgs/Message2";

    TopicRegistrationInfo topic2 =
        masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName2,
            topicMessageType2);

    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    // Delete the first
    masterRegistrationManager.unregisterSubscriber(nodeName, topicName1);

    // TopicSystemState 1 should have no subscribers at all. TopicSystemState 2 will have 1
    // subscriber,
    // and the node will show this subscribed topic.
    assertTrue(topic1.getSubscribers().isEmpty());
    assertEquals(Sets.newHashSet(topic2), node.getSubscribers());
    assertEquals(Sets.newHashSet(node), topic2.getSubscribers());
  }

  /**
   * Register the same subscriber twice. Should give same objects.
   * 
   * @throws Exception
   */
  @Test
  public void testSubscriberTwice() throws Exception {
    GraphName topicName = GraphName.of("/topic");

    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri = new URI("http://localhost:12345");
    String topicMessageType = "topic/Message";

    TopicRegistrationInfo topic1 =
        masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName,
            topicMessageType);
    NodeRegistrationInfo node1 = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    TopicRegistrationInfo topic2 =
        masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName,
            topicMessageType);
    NodeRegistrationInfo node2 = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    // This is the important test. Nothing should have changed.
    assertTrue(topic1 == topic2);
    assertTrue(node1 == node2);

    // Check everything else to make sure stayed the same.
    assertTrue(topic1.getPublishers().isEmpty());
    assertTrue(node1.hasRegistrations());
    assertTrue(node1.getPublishers().isEmpty());
    assertEquals(Sets.newHashSet(topic1), node1.getSubscribers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node1);
  }

  /**
   * Register the same publisher twice. Should give same objects.
   * 
   * @throws Exception
   */
  @Test
  public void testPublisherTwice() throws Exception {
    GraphName topicName = GraphName.of("/topic");

    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri = new URI("http://localhost:12345");
    String topicMessageType = "topic/Message";

    TopicRegistrationInfo topic1 =
        masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri, topicName,
            topicMessageType);
    NodeRegistrationInfo node1 = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    TopicRegistrationInfo topic2 =
        masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri, topicName,
            topicMessageType);
    NodeRegistrationInfo node2 = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    // This is the important test. Nothing should have changed, the exact
    // objects should come back.
    assertTrue(topic1 == topic2);
    assertTrue(node1 == node2);

    // Check everything else to make sure stayed the same.
    assertTrue(topic1.getSubscribers().isEmpty());
    assertTrue(node1.hasRegistrations());
    assertTrue(node1.getSubscribers().isEmpty());
    assertEquals(Sets.newHashSet(topic1), node1.getPublishers());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node1);
  }

  /**
   * Register only a service.
   * 
   * @throws Exception
   */
  @Test
  public void testRegisterOnlyService() throws Exception {
    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri = new URI("http://localhost:12345");
    GraphName serviceName = GraphName.of("/service");
    URI serviceUri = new URI("http://foo");

    ServiceRegistrationInfo service =
        masterRegistrationManager.registerService(nodeName, nodeSlaveUri, serviceName, serviceUri);

    assertEquals(serviceName, service.getServiceName());

    assertEquals(serviceUri, service.getServiceUri());

    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);
    assertTrue(node.hasRegistrations());
    assertEquals(Sets.newHashSet(service), node.getServices());

    // Make sure only publisher in the topic is the topic we got for the node
    // name
    assertEquals(node, service.getNode());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node);
  }

  /**
   * Register a service and then unregister it. Make sure it goes away.
   * 
   * @throws Exception
   */
  @Test
  public void testRegisterUnregisterService() throws Exception {
    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri = new URI("http://localhost:12345");
    GraphName serviceName = GraphName.of("/service");
    URI serviceUri = new URI("http://foo");

    masterRegistrationManager.registerService(nodeName, nodeSlaveUri, serviceName, serviceUri);

    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    masterRegistrationManager.unregisterService(nodeName, serviceName, serviceUri);

    assertNull(masterRegistrationManager.getServiceRegistrationInfo(serviceName));
    assertNull(masterRegistrationManager.getNodeRegistrationInfo(nodeName));

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node);
  }

  /**
   * Register a service and then unregister it. Have a publisher registered for
   * the the same node. make sure node sticks around.
   * 
   * @throws Exception
   */
  @Test
  public void testRegisterUnregisterServiceWithPublisher() throws Exception {
    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri = new URI("http://localhost:12345");
    GraphName serviceName = GraphName.of("/service");
    URI serviceUri = new URI("http://foo");
    GraphName topicName = GraphName.of("/topic");
    String topicMessageType = "foo/Bar";

    masterRegistrationManager.registerService(nodeName, nodeSlaveUri, serviceName, serviceUri);
    masterRegistrationManager
        .registerPublisher(nodeName, nodeSlaveUri, topicName, topicMessageType);

    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    masterRegistrationManager.unregisterService(nodeName, serviceName, serviceUri);

    assertNull(masterRegistrationManager.getServiceRegistrationInfo(serviceName));
    assertEquals(node, masterRegistrationManager.getNodeRegistrationInfo(nodeName));
    assertTrue(node.getServices().isEmpty());

    // No attempt for node shutdown
    verify(registrationListener, Mockito.never()).onNodeReplacement(node);
  }

  /**
   * Register a service and then force a node replacement. Make sure service
   * goes away.
   * 
   * @throws Exception
   */
  @Test
  public void testRegisterServiceReplaceNode() throws Exception {
    GraphName nodeName = GraphName.of("/node");
    URI nodeSlaveUri1 = new URI("http://localhost:12345");
    GraphName serviceName = GraphName.of("/service");
    URI serviceUri = new URI("http://foo");
    GraphName topicName = GraphName.of("/topic");
    String topicMessageType = "foo/Bar";

    URI nodeSlaveUri2 = new URI("http://localhost:54321");

    masterRegistrationManager.registerService(nodeName, nodeSlaveUri1, serviceName, serviceUri);
    masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri1, topicName,
        topicMessageType);

    NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);

    masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri2, topicName,
        topicMessageType);

    assertNull(masterRegistrationManager.getServiceRegistrationInfo(serviceName));
    assertTrue(masterRegistrationManager.getNodeRegistrationInfo(nodeName).getServices().isEmpty());

    // No attempt for node shutdown
    verify(registrationListener).onNodeReplacement(node);
  }
}
