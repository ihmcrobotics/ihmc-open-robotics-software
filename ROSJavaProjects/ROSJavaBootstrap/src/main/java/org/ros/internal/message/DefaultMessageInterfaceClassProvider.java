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

import com.google.common.annotations.VisibleForTesting;
import com.google.common.collect.Maps;

import java.util.Map;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class DefaultMessageInterfaceClassProvider implements MessageInterfaceClassProvider {

  private final Map<String, Class<?>> cache;

  public DefaultMessageInterfaceClassProvider() {
    cache = Maps.newConcurrentMap();
  }

  @SuppressWarnings("unchecked")
  @Override
  public <T> Class<T> get(String messageType) {
    if (cache.containsKey(messageType)) {
      return (Class<T>) cache.get(messageType);
    }
    try {
      String className = messageType.replace("/", ".");
      Class<T> messageInterfaceClass = (Class<T>) getClass().getClassLoader().loadClass(className);
      cache.put(messageType, messageInterfaceClass);
      return messageInterfaceClass;
    } catch (ClassNotFoundException e) {
      return (Class<T>) RawMessage.class;
    }
  }

  @VisibleForTesting
  <T> void add(String messageType, Class<T> messageInterfaceClass) {
    cache.put(messageType, messageInterfaceClass);
  }
}
