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

import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.field.PrimitiveFieldType;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;

/**
 * Parses message definitions and invokes a {@link MessageDefinitionVisitor} for
 * each field.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageDefinitionParser {

  private final MessageDefinitionVisitor visitor;

  public interface MessageDefinitionVisitor {
    /**
     * Called for each constant in the message definition.
     * 
     * @param type
     *          the type of the constant
     * @param name
     *          the name of the constant
     * @param value
     *          the value of the constant
     */
    void constantValue(String type, String name, String value);

    /**
     * Called for each scalar in the message definition.
     * 
     * @param type
     *          the type of the scalar
     * @param name
     *          the name of the scalar
     */
    void variableValue(String type, String name);

    /**
     * Called for each array in the message definition.
     * 
     * @param type
     *          the type of the array
     * @param size
     *          the size of the array or -1 if the size is unbounded
     * @param name
     *          the name of the array
     */
    void variableList(String type, int size, String name);
  }

  /**
   * @param visitor
   *          the {@link MessageDefinitionVisitor} that will be called for each
   *          field
   */
  public MessageDefinitionParser(MessageDefinitionVisitor visitor) {
    this.visitor = visitor;
  }

  /**
   * Parses the message definition
   * 
   * @param messageType
   *          the type of message defined (e.g. std_msgs/String)
   * @param messageDefinition
   *          the message definition (e.g. "string data")
   */
  public void parse(String messageType, String messageDefinition) {
    Preconditions.checkNotNull(messageType);
    Preconditions.checkNotNull(messageDefinition);
    BufferedReader reader = new BufferedReader(new StringReader(messageDefinition));
    String line;
    try {
      while (true) {
        line = reader.readLine();
        if (line == null) {
          break;
        }
        line = line.trim();
        if (line.startsWith("#")) {
          continue;
        }
        if (line.length() > 0) {
          parseField(messageType, line);
        }
      }
    } catch (IOException e) {
      throw new RosRuntimeException(e);
    }
  }

  private void parseField(String messageType, String fieldDefinition) {
    // TODO(damonkohler): Regex input validation.
    String[] typeAndName = fieldDefinition.split("\\s+", 2);
    Preconditions.checkState(typeAndName.length == 2,
        String.format("Invalid field definition: \"%s\"", fieldDefinition));
    String type = typeAndName[0];
    String name = typeAndName[1];
    String value = null;
    if (name.contains("=") && (!name.contains("#") || name.indexOf('#') > name.indexOf('='))) {
      String[] nameAndValue = name.split("=", 2);
      name = nameAndValue[0].trim();
      value = nameAndValue[1].trim();
    } else if (name.contains("#")) {
      // Stripping comments from constants is deferred until we also know the
      // type since strings are handled differently.
      Preconditions.checkState(!name.startsWith("#"), String.format(
          "Fields must define a name. Field definition in %s was: \"%s\"", messageType,
          fieldDefinition));
      name = name.substring(0, name.indexOf('#'));
      name = name.trim();
    }
    boolean array = false;
    int size = -1;
    if (type.endsWith("]")) {
      int leftBracketIndex = type.lastIndexOf('[');
      int rightBracketIndex = type.lastIndexOf(']');
      array = true;
      if (rightBracketIndex - leftBracketIndex > 1) {
        size = Integer.parseInt(type.substring(leftBracketIndex + 1, rightBracketIndex));
      }
      type = type.substring(0, leftBracketIndex);
    }
    if (type.equals("Header")) {
      // The header field is treated as though it were a built-in and silently
      // expanded to "std_msgs/Header."
      Preconditions.checkState(name.equals("header"), "Header field must be named \"header.\"");
      type = "std_msgs/Header";
    } else if (!PrimitiveFieldType.existsFor(type) && !type.contains("/")) {
      // Handle package relative message names.
      type = messageType.substring(0, messageType.lastIndexOf('/') + 1) + type;
    }
    if (value != null) {
      if (array) {
        // TODO(damonkohler): Handle array constants?
        throw new UnsupportedOperationException("Array constants are not supported.");
      }
      // Comments inline with string constants are treated as data.
      if (!type.equals(PrimitiveFieldType.STRING.getName()) && value.contains("#")) {
        Preconditions.checkState(!value.startsWith("#"), "Constants must define a value.");
        value = value.substring(0, value.indexOf('#'));
        value = value.trim();
      }
      visitor.constantValue(type, name, value);
    } else {
      if (array) {
        visitor.variableList(type, size, name);
      } else {
        visitor.variableValue(type, name);
      }
    }
  }
}
