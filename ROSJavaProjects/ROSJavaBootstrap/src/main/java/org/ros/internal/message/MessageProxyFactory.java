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

import org.ros.internal.message.context.MessageContext;
import org.ros.internal.message.context.MessageContextProvider;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;

import java.lang.reflect.Proxy;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageProxyFactory {

  // We can't use the constant here since the rosjava_messages package depends
  // on rosjava_bootstrap.
  private static final String HEADER_MESSAGE_TYPE = "std_msgs/Header";
  private static final String SEQUENCE_FIELD_NAME = "seq";
  private static final AtomicInteger SEQUENCE_NUMBER = new AtomicInteger(0);

  private final MessageInterfaceClassProvider messageInterfaceClassProvider;
  private final MessageContextProvider messageContextProvider;

  public MessageProxyFactory(MessageInterfaceClassProvider messageInterfaceClassProvider,
      MessageFactory messageFactory) {
    this.messageInterfaceClassProvider = messageInterfaceClassProvider;
    messageContextProvider = new MessageContextProvider(messageFactory);
  }

  @SuppressWarnings("unchecked")
  public <T> T newMessageProxy(MessageDeclaration messageDeclaration) {
    Preconditions.checkNotNull(messageDeclaration);
    MessageContext messageContext = messageContextProvider.get(messageDeclaration);
    MessageImpl messageImpl = new MessageImpl(messageContext);
    // Header messages are automatically populated with a monotonically
    // increasing sequence number.
    if (messageImpl.getType().equals(HEADER_MESSAGE_TYPE)) {
      messageImpl.setUInt32(SEQUENCE_FIELD_NAME, SEQUENCE_NUMBER.getAndIncrement());
    }
    Class<T> messageInterfaceClass =
        (Class<T>) messageInterfaceClassProvider.get(messageDeclaration.getType());
    return newProxy(messageInterfaceClass, messageImpl);
  }

  /**
   * @param interfaceClass
   *          the interface class to provide
   * @param messageImpl
   *          the instance to proxy
   * @return a new proxy for {@code implementation} that implements
   *         {@code interfaceClass}
   */
  @SuppressWarnings("unchecked")
  private <T> T newProxy(Class<T> interfaceClass, final MessageImpl messageImpl) {
    ClassLoader classLoader = messageImpl.getClass().getClassLoader();
    Class<?>[] interfaces = new Class<?>[] { interfaceClass, GetInstance.class };
    MessageProxyInvocationHandler invocationHandler =
        new MessageProxyInvocationHandler(messageImpl);
    return (T) Proxy.newProxyInstance(classLoader, interfaces, invocationHandler);
  }
}
