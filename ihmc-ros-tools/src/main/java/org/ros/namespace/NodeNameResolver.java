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

package org.ros.namespace;

import org.ros.node.Node;

/**
 * Resolver for {@link Node} names. {@link Node} namespace must handle the ~name
 * syntax for private names.
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author kwc@willowgarage.com (Ken Conley)
 */
public class NodeNameResolver extends NameResolver {

  private final GraphName privateNamespace;

  /**
   * @param nodeName
   *          the name of the {@link Node}
   * @param defaultResolver
   *          the {@link NameResolver} to use if asked to resolve a non-private
   *          name
   */
  public NodeNameResolver(GraphName nodeName, NameResolver defaultResolver) {
    super(defaultResolver.getNamespace(), defaultResolver.getRemappings());
    this.privateNamespace = nodeName;
  }

  /**
   * @param name
   *          name to resolve
   * @return the name resolved relative to the default or private namespace
   */
  @Override
  public GraphName resolve(GraphName name) {
    GraphName graphName = lookUpRemapping(name);
    if (graphName.isPrivate()) {
      return resolve(privateNamespace, graphName.toRelative());
    }
    return super.resolve(name);
  }

  /**
   * @see #resolve(GraphName)
   */
  @Override
  public GraphName resolve(String name) {
    return resolve(GraphName.of(name));
  }
}
