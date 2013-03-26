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

package org.ros.internal.message.field;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.DefaultMessageDeserializer;
import org.ros.internal.message.DefaultMessageSerializer;
import org.ros.internal.message.Message;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;
import org.ros.message.MessageSerializer;


/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageFieldType implements FieldType {

  private final MessageIdentifier messageIdentifier;
  private final MessageFactory messageFactory;
  private final MessageSerializer<Message> serializer;
  private final MessageDeserializer<Message> deserializer;

  public MessageFieldType(MessageIdentifier messageIdentifier, MessageFactory messageFactory) {
    this.messageIdentifier = messageIdentifier;
    this.messageFactory = messageFactory;
    serializer = new DefaultMessageSerializer();
    deserializer = new DefaultMessageDeserializer<Message>(messageIdentifier, messageFactory);
  }

  public MessageFactory getMessageFactory() {
    return messageFactory;
  }

  @Override
  public Field newVariableValue(String name) {
    return ValueField.newVariable(this, name);
  }

  @Override
  public <T> Field newConstantValue(String name, T value) {
    throw new UnsupportedOperationException();
  }

  @Override
  public Field newVariableList(String name, int size) {
    return ListField.newVariable(this, name);
  }

  @Override
  public <T> T getDefaultValue() {
    return getMessageFactory().newFromType(messageIdentifier.getType());
  }

  @Override
  public String getMd5String() {
    return null;
  }

  @Override
  public String getJavaTypeName() {
    return String.format("%s.%s", messageIdentifier.getPackage(), messageIdentifier.getName());
  }

  @Override
  public int getSerializedSize() {
    throw new UnsupportedOperationException();
  }

  @Override
  public String getName() {
    return messageIdentifier.getType();
  }

  @Override
  public <T> void serialize(T value, ChannelBuffer buffer) {
    serializer.serialize((Message) value, buffer);
  }

  @SuppressWarnings("unchecked")
  @Override
  public Message deserialize(ChannelBuffer buffer) {
    return deserializer.deserialize(buffer);
  }

  @SuppressWarnings("unchecked")
  @Override
  public Void parseFromString(String value) {
    throw new UnsupportedOperationException();
  }

  @Override
  public String toString() {
    return "MessageField<" + messageIdentifier + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((messageIdentifier == null) ? 0 : messageIdentifier.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    MessageFieldType other = (MessageFieldType) obj;
    if (messageIdentifier == null) {
      if (other.messageIdentifier != null)
        return false;
    } else if (!messageIdentifier.equals(other.messageIdentifier))
      return false;
    return true;
  }
}
