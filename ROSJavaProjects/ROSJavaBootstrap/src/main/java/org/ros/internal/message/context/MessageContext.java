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

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import org.ros.internal.message.field.FieldFactory;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;

import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Encapsulates the immutable metadata that describes a message type.
 * <p>
 * Note that this class is not thread safe.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageContext {

  private final MessageDeclaration messageDeclaration;
  private final MessageFactory messageFactory;
  private final Map<String, FieldFactory> fieldFactories;
  private final Map<String, String> fieldGetterNames;
  private final Map<String, String> fieldSetterNames;
  private final List<String> fieldNames;

  public MessageContext(MessageDeclaration messageDeclaration, MessageFactory messageFactory) {
    this.messageDeclaration = messageDeclaration;
    this.messageFactory = messageFactory;
    this.fieldFactories = Maps.newHashMap();
    this.fieldGetterNames = Maps.newHashMap();
    this.fieldSetterNames = Maps.newHashMap();
    this.fieldNames = Lists.newArrayList();
  }

  public MessageFactory getMessageFactory() {
    return messageFactory;
  }

  public MessageIdentifier getMessageIdentifer() {
    return messageDeclaration.getMessageIdentifier();
  }

  public String getType() {
    return messageDeclaration.getType();
  }

  public String getPackage() {
    return messageDeclaration.getPackage();
  }

  public String getName() {
    return messageDeclaration.getName();
  }

  public String getDefinition() {
    return messageDeclaration.getDefinition();
  }

  public void addFieldFactory(String name, FieldFactory fieldFactory) {
    fieldFactories.put(name, fieldFactory);
    fieldGetterNames.put(name, "get" + getJavaName(name));
    fieldSetterNames.put(name, "set" + getJavaName(name));
    fieldNames.add(name);
  }

  private String getJavaName(String name) {
    String[] parts = name.split("_");
    StringBuilder fieldName = new StringBuilder();
    for (String part : parts) {
      fieldName.append(part.substring(0, 1).toUpperCase() + part.substring(1));
    }
    return fieldName.toString();
  }

  public boolean hasField(String name) {
    // O(1) instead of an O(n) check against the list of field names.
    return fieldFactories.containsKey(name);
  }

  public String getFieldGetterName(String name) {
    return fieldGetterNames.get(name);
  }

  public String getFieldSetterName(String name) {
    return fieldSetterNames.get(name);
  }

  public FieldFactory getFieldFactory(String name) {
    return fieldFactories.get(name);
  }

  /**
   * @return a {@link List} of field names in the order they were added
   */
  public List<String> getFieldNames() {
    return Collections.unmodifiableList(fieldNames);
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((messageDeclaration == null) ? 0 : messageDeclaration.hashCode());
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
    MessageContext other = (MessageContext) obj;
    if (messageDeclaration == null) {
      if (other.messageDeclaration != null)
        return false;
    } else if (!messageDeclaration.equals(other.messageDeclaration))
      return false;
    return true;
  }
}
