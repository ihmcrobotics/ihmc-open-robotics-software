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


import java.util.Collection;

/**
 * Builds new {@link Node}s.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface NodeFactory {

  /**
   * Build a new {@link Node} with the given {@link NodeConfiguration}.
   * 
   * @param configuration
   *          the {@link NodeConfiguration} for the new {@link Node}
   * @return a new {@link Node}
   */
  Node newNode(NodeConfiguration configuration);

  /**
   * Build a new {@link Node} with the given {@link NodeConfiguration} and
   * {@link NodeListener}s.
   * 
   * @param configuration
   *          the {@link NodeConfiguration} for the new {@link Node}
   * @param listeners
   *          a collection of {@link NodeListener} instances which will be
   *          registered with the node (can be {@code null})
   * @return a new {@link Node}
   */
  Node newNode(NodeConfiguration configuration, Collection<NodeListener> listeners);
}
