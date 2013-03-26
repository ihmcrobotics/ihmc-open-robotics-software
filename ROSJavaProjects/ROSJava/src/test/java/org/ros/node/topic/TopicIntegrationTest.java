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

package org.ros.node.topic;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;
import org.ros.RosTest;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.internal.message.topic.TopicMessageFactory;
import org.ros.internal.node.topic.DefaultSubscriber;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import java.net.InetSocketAddress;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * Make sure publishers can talk with subscribers over a network connection.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TopicIntegrationTest extends RosTest {

  private static final int QUEUE_CAPACITY = 128;

  private final std_msgs.String expectedMessage;

  public TopicIntegrationTest() {
    MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    TopicMessageFactory topicMessageFactory = new TopicMessageFactory(messageDefinitionProvider);
    expectedMessage = topicMessageFactory.newFromType(std_msgs.String._TYPE);
    expectedMessage.setData("Would you like to play a game?");
  }

  @Test
  public void testOnePublisherToOneSubscriber() throws InterruptedException {
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("publisher");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        Publisher<std_msgs.String> publisher =
            connectedNode.newPublisher("foo", std_msgs.String._TYPE);
        publisher.setLatchMode(true);
        publisher.publish(expectedMessage);
      }
    }, nodeConfiguration);

    final CountDownLatch messageReceived = new CountDownLatch(1);
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("subscriber");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        Subscriber<std_msgs.String> subscriber =
            connectedNode.newSubscriber("foo", std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
          @Override
          public void onNewMessage(std_msgs.String message) {
            assertEquals(expectedMessage, message);
            messageReceived.countDown();
          }
        }, QUEUE_CAPACITY);
      }
    }, nodeConfiguration);

    assertTrue(messageReceived.await(1, TimeUnit.SECONDS));
  }

  /**
   * This is a regression test.
   * 
   * @see <a
   *      href="http://answers.ros.org/question/3591/rosjava-subscriber-unreliable">bug
   *      report</a>
   * 
   * @throws InterruptedException
   */
  @Test
  public void testSubscriberStartsBeforePublisher() throws InterruptedException {
    final CountDownSubscriberListener<std_msgs.String> subscriberListener =
        CountDownSubscriberListener.newDefault();
    final CountDownLatch messageReceived = new CountDownLatch(1);
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("subscriber");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        Subscriber<std_msgs.String> subscriber =
            connectedNode.newSubscriber("foo", std_msgs.String._TYPE);
        subscriber.addSubscriberListener(subscriberListener);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
          @Override
          public void onNewMessage(std_msgs.String message) {
            assertEquals(expectedMessage, message);
            messageReceived.countDown();
          }
        }, QUEUE_CAPACITY);
      }
    }, nodeConfiguration);

    subscriberListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);

    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("publisher");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        Publisher<std_msgs.String> publisher =
            connectedNode.newPublisher("foo", std_msgs.String._TYPE);
        publisher.setLatchMode(true);
        publisher.publish(expectedMessage);
      }
    }, nodeConfiguration);

    assertTrue(messageReceived.await(1, TimeUnit.SECONDS));
  }

  @Test
  public void testAddDisconnectedPublisher() {
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("subscriber");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        DefaultSubscriber<std_msgs.String> subscriber =
            (DefaultSubscriber<std_msgs.String>) connectedNode.<std_msgs.String>newSubscriber(
                "foo", std_msgs.String._TYPE);
        try {
          subscriber.addPublisher(PublisherIdentifier.newFromStrings("foo", "http://foo", "foo"),
              new InetSocketAddress(1234));
          fail();
        } catch (RuntimeException e) {
          // Connecting to a disconnected publisher should fail.
        }
      }
    }, nodeConfiguration);
  }

  private final class Listener implements MessageListener<test_ros.TestHeader> {
    private final CountDownLatch latch = new CountDownLatch(10);

    private test_ros.TestHeader lastMessage;

    @Override
    public void onNewMessage(test_ros.TestHeader message) {
      int seq = message.getHeader().getSeq();
      long stamp = message.getHeader().getStamp().totalNsecs();
      if (lastMessage != null) {
        int lastSeq = lastMessage.getHeader().getSeq();
        long lastStamp = lastMessage.getHeader().getStamp().totalNsecs();
        assertTrue(String.format("message seq %d <= previous seq %d", seq, lastSeq), seq > lastSeq);
        assertTrue(String.format("message stamp %d <= previous stamp %d", stamp, lastStamp),
            stamp > lastStamp);
      }
      lastMessage = message;
      latch.countDown();
    }

    public boolean await(long timeout, TimeUnit unit) throws InterruptedException {
      return latch.await(timeout, unit);
    }
  }

  @Test
  public void testHeader() throws InterruptedException {
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("publisher");
      }

      @Override
      public void onStart(final ConnectedNode connectedNode) {
        final Publisher<test_ros.TestHeader> publisher =
            connectedNode.newPublisher("foo", test_ros.TestHeader._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {
          @Override
          public void loop() throws InterruptedException {
            test_ros.TestHeader message =
                connectedNode.getTopicMessageFactory().newFromType(test_ros.TestHeader._TYPE);
            message.getHeader().setStamp(connectedNode.getCurrentTime());
            publisher.publish(message);
            // There needs to be some time between messages in order to
            // guarantee that the timestamp increases.
            Thread.sleep(1);
          }
        });
      }
    }, nodeConfiguration);

    final Listener listener = new Listener();
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("subscriber");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        Subscriber<test_ros.TestHeader> subscriber =
            connectedNode.newSubscriber("foo", test_ros.TestHeader._TYPE);
        subscriber.addMessageListener(listener, QUEUE_CAPACITY);
      }
    }, nodeConfiguration);

    assertTrue(listener.await(1, TimeUnit.SECONDS));
  }
}
