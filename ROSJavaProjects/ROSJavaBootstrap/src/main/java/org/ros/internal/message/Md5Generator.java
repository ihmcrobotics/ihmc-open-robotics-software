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

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.ros.internal.message.definition.MessageDefinitionParser;
import org.ros.internal.message.definition.MessageDefinitionTupleParser;
import org.ros.internal.message.definition.MessageDefinitionParser.MessageDefinitionVisitor;

import org.apache.commons.codec.digest.DigestUtils;
import org.ros.internal.message.field.PrimitiveFieldType;
import org.ros.message.MessageDefinitionProvider;

import java.util.List;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class Md5Generator {

  private final MessageDefinitionProvider messageDefinitionProvider;

  public Md5Generator(MessageDefinitionProvider messageDefinitionProvider) {
    this.messageDefinitionProvider = messageDefinitionProvider;
  }

  public String generate(String messageType) {
    String messageDefinition = messageDefinitionProvider.get(messageType);
    Preconditions.checkNotNull(messageDefinition, "No definition for message type: " + messageType);
    List<String> parts = MessageDefinitionTupleParser.parse(messageDefinition, -1);
    StringBuilder text = new StringBuilder();
    for (String part : parts) {
      text.append(generateText(messageType, part));
    }
    return DigestUtils.md5Hex(text.toString());
  }

  private String generateText(String messageType, String messageDefinition) {
    final List<String> constants = Lists.newArrayList();
    final List<String> variables = Lists.newArrayList();
    MessageDefinitionVisitor visitor = new MessageDefinitionVisitor() {
      @Override
      public void variableValue(String type, String name) {
        if (!PrimitiveFieldType.existsFor(type)) {
          type = generate(type);
        }
        variables.add(String.format("%s %s\n", type, name));
      }

      @Override
      public void variableList(String type, int size, String name) {
        if (!PrimitiveFieldType.existsFor(type)) {
          String md5Checksum = generate(type);
          variables.add(String.format("%s %s\n", md5Checksum, name));
        } else {
          if (size != -1) {
            variables.add(String.format("%s[%d] %s\n", type, size, name));
          } else {
            variables.add(String.format("%s[] %s\n", type, name));
          }
        }
      }

      @Override
      public void constantValue(String type, String name, String value) {
        constants.add(String.format("%s %s=%s\n", type, name, value));
      }
    };
    MessageDefinitionParser messageDefinitionParser = new MessageDefinitionParser(visitor);
    messageDefinitionParser.parse(messageType, messageDefinition);
    String text = "";
    for (String constant : constants) {
      text += constant;
    }
    for (String variable : variables) {
      text += variable;
    }
    return text.trim();
  }
}
