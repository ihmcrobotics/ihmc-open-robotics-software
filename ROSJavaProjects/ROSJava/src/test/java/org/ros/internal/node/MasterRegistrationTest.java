// Copyright 2011 Google Inc. All Rights Reserved.

package org.ros.internal.node;

import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.ros.RosCore;
import org.ros.RosTest;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.CountDownPublisherListener;
import org.ros.node.topic.Publisher;

import java.io.IOException;
import java.net.URISyntaxException;
import java.util.concurrent.TimeUnit;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MasterRegistrationTest extends RosTest {

  private CountDownPublisherListener<std_msgs.String> publisherListener;
  private Publisher<std_msgs.String> publisher;

  @Test
  public void testRegisterPublisher() throws InterruptedException {
    publisherListener = CountDownPublisherListener.newDefault();
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("node");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher("topic", std_msgs.String._TYPE);
        publisher.addListener(publisherListener);
      }
    }, nodeConfiguration);
    assertTrue(publisherListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS));
    publisher.shutdown();
    assertTrue(publisherListener.awaitMasterUnregistrationSuccess(1, TimeUnit.SECONDS));
  }

  @Test
  public void testRegisterPublisherRetries() throws InterruptedException, IOException,
      URISyntaxException {
    int port = rosCore.getUri().getPort();
    publisherListener = CountDownPublisherListener.newDefault();
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("node");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        rosCore.shutdown();
        ((DefaultNode) connectedNode).getRegistrar().setRetryDelay(1, TimeUnit.MILLISECONDS);
        publisher = connectedNode.newPublisher("topic", std_msgs.String._TYPE);
        publisher.addListener(publisherListener);
      }
    }, nodeConfiguration);

    assertTrue(publisherListener.awaitMasterRegistrationFailure(1, TimeUnit.SECONDS));
    rosCore = RosCore.newPrivate(port);
    rosCore.start();
    assertTrue(rosCore.awaitStart(1, TimeUnit.SECONDS));
    assertTrue(publisherListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS));
    publisher.shutdown();
    assertTrue(publisherListener.awaitMasterUnregistrationSuccess(1, TimeUnit.SECONDS));
  }
}
