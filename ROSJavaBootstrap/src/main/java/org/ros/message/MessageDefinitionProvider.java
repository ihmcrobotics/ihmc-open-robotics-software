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

package org.ros.message;

import java.util.Collection;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface MessageDefinitionProvider {

  /**
   * @param messageType
   *          the type of message definition to provide
   * @return the message definition for the specified type
   */
  String get(String messageType);

  /**
   * @param messageType
   *          the type of message definition to provide
   * @return {@code true} if the definition for the specified type is available,
   *         {@code false} otherwise
   */
  boolean has(String messageType);

  Collection<String> getPackages();

  /**
   * @param pkg
   *          the name of the package to filter on
   * @return the {@link MessageIdentifier}s for all messages defined in the
   *         specified package
   */
  Collection<MessageIdentifier> getMessageIdentifiersByPackage(String pkg);
}
