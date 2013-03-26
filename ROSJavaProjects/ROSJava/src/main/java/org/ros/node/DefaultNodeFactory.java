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

package org.ros.node;

import org.ros.internal.node.DefaultNode;

import org.ros.concurrent.SharedScheduledExecutorService;

import java.util.Collection;
import java.util.concurrent.ScheduledExecutorService;

/**
 * Constructs {@link DefaultNode}s.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class DefaultNodeFactory implements NodeFactory {

  private final ScheduledExecutorService scheduledExecutorService;

  public DefaultNodeFactory(ScheduledExecutorService scheduledExecutorService) {
    this.scheduledExecutorService = new SharedScheduledExecutorService(scheduledExecutorService);
  }

  @Override
  public Node newNode(NodeConfiguration nodeConfiguration, Collection<NodeListener> listeners) {
    return new DefaultNode(nodeConfiguration, listeners, scheduledExecutorService);
  }

  @Override
  public Node newNode(NodeConfiguration nodeConfiguration) {
    return newNode(nodeConfiguration, null);
  }
}
