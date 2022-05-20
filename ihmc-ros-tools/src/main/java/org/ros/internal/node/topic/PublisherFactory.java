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

package org.ros.internal.node.topic;

import org.ros.internal.node.server.NodeIdentifier;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.namespace.GraphName;
import org.ros.node.topic.DefaultPublisherListener;
import org.ros.node.topic.Publisher;

import java.util.concurrent.ScheduledExecutorService;

/**
 * A factory for {@link Publisher} instances.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class PublisherFactory {

  private final TopicParticipantManager topicParticipantManager;
  private final MessageFactory messageFactory;
  private final ScheduledExecutorService executorService;
  private final NodeIdentifier nodeIdentifier;
  private final Object mutex;

  public PublisherFactory(NodeIdentifier nodeIdentifier,
      TopicParticipantManager topicParticipantManager, MessageFactory messageFactory,
      ScheduledExecutorService executorService) {
    this.nodeIdentifier = nodeIdentifier;
    this.topicParticipantManager = topicParticipantManager;
    this.messageFactory = messageFactory;
    this.executorService = executorService;
    mutex = new Object();
  }

  /**
   * Gets or creates a {@link Publisher} instance. {@link Publisher}s are cached
   * and reused per topic. When a new {@link Publisher} is generated, it is
   * registered with the master.
   * 
   * @param <T>
   *          the message type associated with the {@link Publisher}
   * @param topicDeclaration
   *          {@link TopicDeclaration} that is being published
   * @param messageSerializer
   *          the {@link MessageSerializer} used for published messages
   * @return a new or cached {@link Publisher} instance
   */
  @SuppressWarnings("unchecked")
  public <T> Publisher<T> newOrExisting(TopicDeclaration topicDeclaration,
      MessageSerializer<T> messageSerializer) {
    GraphName topicName = topicDeclaration.getName();
    synchronized (mutex) {
      if (topicParticipantManager.hasPublisher(topicName)) {
        return (DefaultPublisher<T>) topicParticipantManager.getPublisher(topicName);
      } else {
        DefaultPublisher<T> publisher =
            new DefaultPublisher<T>(nodeIdentifier, topicDeclaration, messageSerializer,
                messageFactory, executorService);
        publisher.addListener(new DefaultPublisherListener<T>() {
          @Override
          public void onNewSubscriber(Publisher<T> publisher,
              SubscriberIdentifier subscriberIdentifier) {
            topicParticipantManager.addPublisherConnection((DefaultPublisher<T>) publisher,
                subscriberIdentifier);
          }

          @Override
          public void onShutdown(Publisher<T> publisher) {
            topicParticipantManager.removePublisher((DefaultPublisher<T>) publisher);
          }
        });
        topicParticipantManager.addPublisher(publisher);
        return publisher;
      }
    }
  }
}
