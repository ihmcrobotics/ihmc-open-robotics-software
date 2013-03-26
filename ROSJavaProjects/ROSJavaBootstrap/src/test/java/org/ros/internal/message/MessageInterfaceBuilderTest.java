/*
 * Copyright (C) 2012 Google Inc.
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

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;
import org.ros.internal.message.topic.TopicDefinitionResourceProvider;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageInterfaceBuilderTest {

  private TopicDefinitionResourceProvider topicDefinitionResourceProvider;
  private MessageFactory messageFactory;

  @Before
  public void before() {
    topicDefinitionResourceProvider = new TopicDefinitionResourceProvider();
    messageFactory = new DefaultMessageFactory(topicDefinitionResourceProvider);
  }

  @Test
  public void testDuplicateFieldNames() {
    MessageInterfaceBuilder builder = new MessageInterfaceBuilder();
    builder.setPackageName("foo");
    builder.setInterfaceName("bar");
    builder.setMessageDeclaration(MessageDeclaration.of("foo/bar", "int32 foo\nint32 Foo"));
    builder.setAddConstantsAndMethods(true);
    String result = builder.build(messageFactory);
    assertEquals("package foo;\n\n"
        + "public interface bar extends org.ros.internal.message.Message {\n"
        + "  static final java.lang.String _TYPE = \"foo/bar\";\n"
        + "  static final java.lang.String _DEFINITION = \"int32 foo\\nint32 Foo\";\n"
        + "  int getFoo();\n" + "  void setFoo(int value);\n" + "}\n", result);
  }
}
