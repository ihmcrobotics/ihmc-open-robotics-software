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

import org.ros.exception.RosRuntimeException;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author kwc@willowgarage.com (Ken Conley)
 */
public class NameResolver {

  private final GraphName namespace;
  private final Map<GraphName, GraphName> remappings;

  public static NameResolver newFromNamespace(GraphName namespace) {
    return new NameResolver(namespace, new HashMap<GraphName, GraphName>());
  }

  public static NameResolver newFromNamespace(String namespace) {
    return newFromNamespace(GraphName.of(namespace));
  }

  public static NameResolver newRoot() {
    return newFromNamespace(GraphName.root());
  }

  public static NameResolver newRootFromRemappings(Map<GraphName, GraphName> remappings) {
    return new NameResolver(GraphName.root(), remappings);
  }

  public static NameResolver newFromNamespaceAndRemappings(String namespace,
      Map<GraphName, GraphName> remappings) {
    return new NameResolver(GraphName.of(namespace), remappings);
  }

  public NameResolver(GraphName namespace, Map<GraphName, GraphName> remappings) {
    this.remappings = Collections.unmodifiableMap(remappings);
    this.namespace = namespace;
  }

  public GraphName getNamespace() {
    return namespace;
  }

  /**
   * Resolve name relative to namespace. If namespace is not global, it will
   * first be resolved to a global name. This method will not resolve private
   * ~names.
   * 
   * This does all remappings of both the namespace and name.
   * 
   * @param namespace
   * @param name
   * @return the fully resolved name relative to the given namespace
   */
  public GraphName resolve(GraphName namespace, GraphName name) {
    GraphName remappedNamespace = lookUpRemapping(namespace);
    if (!remappedNamespace.isGlobal()) {
      throw new IllegalArgumentException(String.format(
          "Namespace %s (remapped from %s) must be global.", remappedNamespace, namespace));
    }
    GraphName remappedName = lookUpRemapping(name);
    if (remappedName.isGlobal()) {
      return remappedName;
    }
    if (remappedName.isRelative()) {
      return remappedNamespace.join(remappedName);
    }
    if (remappedName.isPrivate()) {
      throw new RosRuntimeException("Cannot resolve ~private names in arbitrary namespaces.");
    }
    throw new RosRuntimeException("Unable to resolve graph name: " + name);
  }

  /**
   * @see #resolve(GraphName, GraphName)
   */
  public GraphName resolve(String namespace, String name) {
    return resolve(GraphName.of(namespace), GraphName.of(name));
  }

  /**
   * @see #resolve(GraphName, GraphName)
   */
  public GraphName resolve(GraphName namespace, String name) {
    return resolve(namespace, GraphName.of(name));
  }

  /**
   * @see #resolve(GraphName, GraphName)
   */
  public GraphName resolve(String namespace, GraphName name) {
    return resolve(GraphName.of(namespace), name);
  }

  /**
   * @param name
   *          name to resolve
   * @return the name resolved relative to the default namespace
   */
  public GraphName resolve(GraphName name) {
    return resolve(namespace, name);
  }

  /**
   * @see #resolve(GraphName)
   */
  public GraphName resolve(String name) {
    return resolve(GraphName.of(name));
  }

  /**
   * @return remappings
   */
  public Map<GraphName, GraphName> getRemappings() {
    return remappings;
  }

  /**
   * Construct a new child {@link NameResolver} with the same remappings as this
   * {@link NameResolver}. The namespace of the new child {@link NameResolver}
   * will be the resolved in this namespace.
   * 
   * @param namespace
   *          the namespace of the child {@link NameResolver} relative to this
   *          {@link NameResolver}'s namespace
   * @return a new child {@link NameResolver} whose namespace is relative to the
   *         parent {@link NameResolver}'s namespace
   */
  public NameResolver newChild(GraphName namespace) {
    return new NameResolver(resolve(namespace), remappings);
  }

  /**
   * @see #newChild(GraphName)
   */
  public NameResolver newChild(String namespace) {
    return newChild(GraphName.of(namespace));
  }

  protected GraphName lookUpRemapping(GraphName name) {
    GraphName remappedName = name;
    if (remappings.containsKey(name)) {
      remappedName = remappings.get(name);
    }
    return remappedName;
  }
}
