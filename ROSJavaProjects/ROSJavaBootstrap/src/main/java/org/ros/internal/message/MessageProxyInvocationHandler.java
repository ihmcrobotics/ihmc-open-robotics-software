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

import org.ros.internal.message.field.Field;
import org.ros.internal.message.field.MessageFields;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageProxyInvocationHandler implements InvocationHandler {

  private final MessageImpl messageImpl;

  MessageProxyInvocationHandler(MessageImpl messageImpl) {
    this.messageImpl = messageImpl;
  }

  @Override
  public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
    String methodName = method.getName();
    MessageFields mesageFields = messageImpl.getMessageFields();
    Field getterField = mesageFields.getGetterField(methodName);
    if (getterField != null) {
      return getterField.getValue();
    }
    Field setterField = mesageFields.getSetterField(methodName);
    if (setterField != null) {
      setterField.setValue(args[0]);
      return null;
    }
    return method.invoke(messageImpl, args);
  }
}