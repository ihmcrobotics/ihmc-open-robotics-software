/*
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership. The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.internal.node.xmlrpc;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.TimingOutCallback;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.common.TypeConverter;
import org.apache.xmlrpc.common.TypeConverterFactory;
import org.apache.xmlrpc.common.TypeConverterFactoryImpl;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.lang.reflect.UndeclaredThrowableException;

/**
 * Modified version of {@link org.apache.xmlrpc.client.util.ClientFactory} that
 * requires timeouts in calls.
 * 
 * @param <T>
 *          the type of {@link XmlRpcEndpoint} to create clients for
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class XmlRpcClientFactory<T extends org.ros.internal.node.xmlrpc.XmlRpcEndpoint> {

  private final XmlRpcClient client;
  private final TypeConverterFactory typeConverterFactory;

  private boolean objectMethodLocal;

  /**
   * Creates a new instance.
   * 
   * @param pClient
   *          A fully configured XML-RPC client, which is used internally to
   *          perform XML-RPC calls.
   * @param pTypeConverterFactory
   *          Creates instances of {@link TypeConverterFactory}, which are used
   *          to transform the result object in its target representation.
   */
  public XmlRpcClientFactory(XmlRpcClient pClient, TypeConverterFactory pTypeConverterFactory) {
    typeConverterFactory = pTypeConverterFactory;
    client = pClient;
  }

  /**
   * Creates a new instance. Shortcut for
   * 
   * <pre>
   * new ClientFactory(pClient, new TypeConverterFactoryImpl());
   * </pre>
   * 
   * @param pClient
   *          A fully configured XML-RPC client, which is used internally to
   *          perform XML-RPC calls.
   * @see TypeConverterFactoryImpl
   */
  public XmlRpcClientFactory(XmlRpcClient pClient) {
    this(pClient, new TypeConverterFactoryImpl());
  }

  /**
   * Returns the factories client.
   */
  public XmlRpcClient getClient() {
    return client;
  }

  /**
   * Returns, whether a method declared by the {@link Object Object class} is
   * performed by the local object, rather than by the server. Defaults to true.
   */
  public boolean isObjectMethodLocal() {
    return objectMethodLocal;
  }

  /**
   * Sets, whether a method declared by the {@link Object Object class} is
   * performed by the local object, rather than by the server. Defaults to true.
   */
  public void setObjectMethodLocal(boolean pObjectMethodLocal) {
    objectMethodLocal = pObjectMethodLocal;
  }

  /**
   * Creates an object, which is implementing the given interface. The objects
   * methods are internally calling an XML-RPC server by using the factories
   * client.
   * 
   * @param pClassLoader
   *          The class loader, which is being used for loading classes, if
   *          required.
   * @param pClass
   *          Interface, which is being implemented.
   * @param pRemoteName
   *          Handler name, which is being used when calling the server. This is
   *          used for composing the method name. For example, if
   *          <code>pRemoteName</code> is "Foo" and you want to invoke the
   *          method "bar" in the handler, then the full method name would be
   *          "Foo.bar".
   */
  public Object newInstance(ClassLoader pClassLoader, final Class<T> pClass,
      final String pRemoteName, final int timeout) {
    return Proxy.newProxyInstance(pClassLoader, new Class[] { pClass }, new InvocationHandler() {
      @Override
      public Object invoke(Object pProxy, Method pMethod, Object[] pArgs) throws Throwable {
        if (isObjectMethodLocal() && pMethod.getDeclaringClass().equals(Object.class)) {
          return pMethod.invoke(pProxy, pArgs);
        }
        final String methodName;
        if (pRemoteName == null || pRemoteName.length() == 0) {
          methodName = pMethod.getName();
        } else {
          methodName = pRemoteName + "." + pMethod.getName();
        }
        Object result;
        try {
          TimingOutCallback callback = new TimingOutCallback(timeout);
          client.executeAsync(methodName, pArgs, callback);
          result = callback.waitForResponse();
        } catch (TimingOutCallback.TimeoutException e) {
          throw new XmlRpcTimeoutException(e);
        } catch (InterruptedException e) {
          throw new XmlRpcTimeoutException(e);
        } catch (UndeclaredThrowableException e) {
          throw new RuntimeException(e);
        } catch (XmlRpcException e) {
          Throwable linkedException = e.linkedException;
          if (linkedException == null) {
            throw new RuntimeException(e);
          }
          Class<?>[] exceptionTypes = pMethod.getExceptionTypes();
          for (int i = 0; i < exceptionTypes.length; i++) {
            Class<?> c = exceptionTypes[i];
            if (c.isAssignableFrom(linkedException.getClass())) {
              throw linkedException;
            }
          }
          throw new RuntimeException(linkedException);
        }
        TypeConverter typeConverter =
            typeConverterFactory.getTypeConverter(pMethod.getReturnType());
        return typeConverter.convert(result);
      }
    });
  }
}
