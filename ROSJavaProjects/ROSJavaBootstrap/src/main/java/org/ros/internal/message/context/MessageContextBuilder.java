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

package org.ros.internal.message.context;

import com.google.common.base.Preconditions;

import org.ros.internal.message.definition.MessageDefinitionParser.MessageDefinitionVisitor;
import org.ros.internal.message.field.Field;
import org.ros.internal.message.field.FieldFactory;
import org.ros.internal.message.field.FieldType;
import org.ros.internal.message.field.MessageFieldType;
import org.ros.internal.message.field.PrimitiveFieldType;
import org.ros.message.MessageIdentifier;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
class MessageContextBuilder implements MessageDefinitionVisitor {

  private final MessageContext messageContext;

  public MessageContextBuilder(MessageContext context) {
    this.messageContext = context;
  }

  private FieldType getFieldType(String type) {
    Preconditions.checkArgument(!type.equals(messageContext.getType()),
        "Message definitions may not be self-referential.");
    FieldType fieldType;
    if (PrimitiveFieldType.existsFor(type)) {
      fieldType = PrimitiveFieldType.valueOf(type.toUpperCase());
    } else {
      fieldType =
          new MessageFieldType(MessageIdentifier.of(type), messageContext.getMessageFactory());
    }
    return fieldType;
  }

  @Override
  public void variableValue(String type, final String name) {
    final FieldType fieldType = getFieldType(type);
    messageContext.addFieldFactory(name, new FieldFactory() {
      @Override
      public Field create() {
        return fieldType.newVariableValue(name);
      }
    });
  }

  @Override
  public void variableList(String type, final int size, final String name) {
    final FieldType fieldType = getFieldType(type);
    messageContext.addFieldFactory(name, new FieldFactory() {
      @Override
      public Field create() {
        return fieldType.newVariableList(name, size);
      }
    });
  }

  @Override
  public void constantValue(String type, final String name, final String value) {
    final FieldType fieldType = getFieldType(type);
    messageContext.addFieldFactory(name, new FieldFactory() {
      @Override
      public Field create() {
        return fieldType.newConstantValue(name, fieldType.parseFromString(value));
      }
    });
  }
}