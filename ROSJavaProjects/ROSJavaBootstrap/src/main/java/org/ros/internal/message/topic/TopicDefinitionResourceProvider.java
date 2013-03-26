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

package org.ros.internal.message.topic;

import com.google.common.annotations.VisibleForTesting;

import org.ros.internal.message.StringResourceProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageIdentifier;

import java.util.Collection;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TopicDefinitionResourceProvider implements MessageDefinitionProvider {

  private final StringResourceProvider stringResourceProvider;

  public TopicDefinitionResourceProvider() {
    stringResourceProvider = new StringResourceProvider();
  }

  private String topicTypeToResourceName(String topicType) {
    MessageIdentifier messageIdentifier = MessageIdentifier.of(topicType);
    return String.format("/%s/msg/%s.msg", messageIdentifier.getPackage(),
        messageIdentifier.getName());
  }

  @Override
  public String get(String topicType) {
    return stringResourceProvider.get(topicTypeToResourceName(topicType));
  }

  @Override
  public boolean has(String topicType) {
    return stringResourceProvider.has(topicTypeToResourceName(topicType));
  }

  @VisibleForTesting
  public void add(String topicType, String topicDefinition) {
    stringResourceProvider.addStringToCache(topicTypeToResourceName(topicType), topicDefinition);
  }

  @Override
  public Collection<String> getPackages() {
    throw new UnsupportedOperationException();
  }

  @Override
  public Collection<MessageIdentifier> getMessageIdentifiersByPackage(String pkg) {
    throw new UnsupportedOperationException();
  }
}
