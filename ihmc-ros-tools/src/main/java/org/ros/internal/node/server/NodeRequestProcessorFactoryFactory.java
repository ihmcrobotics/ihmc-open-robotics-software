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

package org.ros.internal.node.server;

import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.server.RequestProcessorFactoryFactory;

/**
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <T>
 */
class NodeRequestProcessorFactoryFactory<T extends org.ros.internal.node.xmlrpc.XmlRpcEndpoint>
    implements RequestProcessorFactoryFactory {

  private final RequestProcessorFactory factory = new NodeRequestProcessorFactory();
  private final T node;

  public NodeRequestProcessorFactoryFactory(T instance) {
    this.node = instance;
  }

  @SuppressWarnings("rawtypes")
  @Override
  public RequestProcessorFactory getRequestProcessorFactory(Class unused) {
    return factory;
  }

  private class NodeRequestProcessorFactory implements RequestProcessorFactory {
    @Override
    public Object getRequestProcessor(XmlRpcRequest xmlRpcRequest) {
      return node;
    }
  }
}
