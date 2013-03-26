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

package org.ros.internal.message.definition;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import java.util.List;

/**
 * Splits message definitions tuples (e.g. service definitions) into separate
 * message definitions.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageDefinitionTupleParser {

  private static final String SEPARATOR = "---";

  /**
   * Splits the message definition tuple into a {@link List} of message
   * definitions. Split message definitions may be empty (e.g. std_srvs/Empty).
   * 
   * @param definition
   *          the message definition tuple
   * @param size
   *          the expected tuple size, or -1 to ignore this requirement
   * @return a {@link List} of the specified size
   */
  public static List<String> parse(String definition, int size) {
    Preconditions.checkNotNull(definition);
    List<String> definitions = Lists.newArrayList();
    StringBuilder current = new StringBuilder();
    for (String line : definition.split("\n")) {
      if (line.startsWith(SEPARATOR)) {
        definitions.add(current.toString());
        current = new StringBuilder();
        continue;
      }
      current.append(line);
      current.append("\n");
    }
    if (current.length() > 0) {
      current.deleteCharAt(current.length() - 1);
    }
    definitions.add(current.toString());
    Preconditions.checkState(size == -1 || definitions.size() <= size,
        String.format("Message tuple exceeds expected size: %d > %d", definitions.size(), size));
    while (definitions.size() < size) {
      definitions.add("");
    }
    return definitions;
  }
}
