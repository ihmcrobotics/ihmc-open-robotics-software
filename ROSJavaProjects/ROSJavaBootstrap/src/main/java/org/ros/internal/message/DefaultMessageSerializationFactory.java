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

package org.ros.internal.message;

import org.ros.internal.message.service.ServiceRequestMessageFactory;
import org.ros.internal.message.service.ServiceResponseMessageFactory;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;
import org.ros.message.MessageSerializationFactory;
import org.ros.message.MessageSerializer;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class DefaultMessageSerializationFactory implements MessageSerializationFactory {

  private final MessageFactory topicMessageFactory;
  private final ServiceRequestMessageFactory serviceRequestMessageFactory;
  private final ServiceResponseMessageFactory serviceResponseMessageFactory;

  public DefaultMessageSerializationFactory(MessageDefinitionProvider messageDefinitionProvider) {
    topicMessageFactory = new DefaultMessageFactory(messageDefinitionProvider);
    serviceRequestMessageFactory = new ServiceRequestMessageFactory(messageDefinitionProvider);
    serviceResponseMessageFactory = new ServiceResponseMessageFactory(messageDefinitionProvider);
  }

  @SuppressWarnings("unchecked")
  @Override
  public <T> MessageSerializer<T> newMessageSerializer(String messageType) {
    return (MessageSerializer<T>) new DefaultMessageSerializer();
  }

  @Override
  public <T> MessageDeserializer<T> newMessageDeserializer(String messageType) {
    return new DefaultMessageDeserializer<T>(MessageIdentifier.of(messageType),
        topicMessageFactory);
  }

  @SuppressWarnings("unchecked")
  @Override
  public <T> MessageSerializer<T> newServiceRequestSerializer(String serviceType) {
    return (MessageSerializer<T>) new DefaultMessageSerializer();
  }

  @Override
  public <T> org.ros.message.MessageDeserializer<T>
      newServiceRequestDeserializer(String serviceType) {
    return new DefaultMessageDeserializer<T>(MessageIdentifier.of(serviceType),
        serviceRequestMessageFactory);
  }

  @SuppressWarnings("unchecked")
  @Override
  public <T> org.ros.message.MessageSerializer<T> newServiceResponseSerializer(String serviceType) {
    return (MessageSerializer<T>) new DefaultMessageSerializer();
  }

  @Override
  public <T> org.ros.message.MessageDeserializer<T> newServiceResponseDeserializer(
      String serviceType) {
    return new DefaultMessageDeserializer<T>(MessageIdentifier.of(serviceType),
        serviceResponseMessageFactory);
  }
}
