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

import org.ros.internal.transport.ConnectionHeader;
import org.ros.master.client.TopicSystemState;
import org.ros.namespace.GraphName;

import java.util.List;

/**
 * Base definition of a {@link TopicSystemState}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public abstract class DefaultTopicParticipant implements TopicParticipant {

  private final TopicDeclaration topicDeclaration;

  public DefaultTopicParticipant(TopicDeclaration topicDeclaration) {
    this.topicDeclaration = topicDeclaration;
  }

  /**
   * @return the {@link TopicDeclaration} of this {@link TopicParticipant}
   */
  public TopicDeclaration getTopicDeclaration() {
    return topicDeclaration;
  }

  public List<String> getTopicDeclarationAsList() {
    return topicDeclaration.toList();
  }

  @Override
  public GraphName getTopicName() {
    return topicDeclaration.getName();
  }

  @Override
  public String getTopicMessageType() {
    return topicDeclaration.getMessageType();
  }

  /**
   * @return the connection header for the {@link TopicSystemState}
   */
  public ConnectionHeader getTopicDeclarationHeader() {
    return topicDeclaration.toConnectionHeader();
  }

  /**
   * Signal that the {@link TopicSystemState} successfully registered with the master.
   */
  public abstract void signalOnMasterRegistrationSuccess();

  /**
   * Signal that the {@link TopicSystemState} failed to register with the master.
   */
  public abstract void signalOnMasterRegistrationFailure();

  /**
   * Signal that the {@link TopicSystemState} successfully unregistered with the master.
   */
  public abstract void signalOnMasterUnregistrationSuccess();

  /**
   * Signal that the {@link TopicSystemState} failed to unregister with the master.
   */
  public abstract void signalOnMasterUnregistrationFailure();
}
