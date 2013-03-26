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

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;

import org.ros.exception.RosRuntimeException;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Pattern;

/**
 * ROS graph resource name.
 * 
 * @see <a href="http://www.ros.org/wiki/Names">Names documentation</a>
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class GraphName {

  @VisibleForTesting
  static final String ANONYMOUS_PREFIX = "anonymous_";

  private static final String ROOT = "/";
  private static final String SEPARATOR = "/";

  // TODO(damonkohler): Why make empty names valid?
  /**
   * Graph names must match this pattern to be valid.
   * <p>
   * Note that empty graph names are considered valid.
   */
  private static final Pattern VALID_GRAPH_NAME_PATTERN = Pattern
      .compile("^([\\~\\/A-Za-z][\\w_\\/]*)?$");

  private static AtomicInteger anonymousCounter = new AtomicInteger();

  private final String name;

  // TODO(damonkohler): This is not safe across multiple hosts/processes.
  // Instead, try to use the same algorithm as in cpp and Python.
  /**
   * Creates an anonymous {@link GraphName}.
   * 
   * @return a new {@link GraphName} suitable for creating an anonymous node
   */
  public static GraphName newAnonymous() {
    return new GraphName(ANONYMOUS_PREFIX + anonymousCounter.incrementAndGet());
  }

  /**
   * @return a {@link GraphName} representing the root namespace
   */
  public static GraphName root() {
    return new GraphName(ROOT);
  }

  /**
   * @return an empty {@link GraphName}
   */
  public static GraphName empty() {
    return new GraphName("");
  }

  /**
   * Returns a new {@link GraphName} of the specified {@link #name}.
   * 
   * @param name
   *          the name of this resource
   * @return a new {@link GraphName} for {@link #name}
   */
  public static GraphName of(String name) {
    return new GraphName(canonicalize(name));
  }

  private GraphName(String name) {
    Preconditions.checkNotNull(name);
    this.name = name;
  }

  /**
   * Validate and convert the graph name into its canonical representation.
   * Canonical representations have no trailing slashes and can be global,
   * private, or relative.
   * 
   * @param name
   * @return the canonical name for this {@link GraphName}
   */
  private static String canonicalize(String name) {
    if (!VALID_GRAPH_NAME_PATTERN.matcher(name).matches()) {
      throw new RosRuntimeException("Invalid graph name: " + name);
    }
    // Trim trailing slashes for canonical representation.
    while (!name.equals(GraphName.ROOT) && name.endsWith(SEPARATOR)) {
      name = name.substring(0, name.length() - 1);
    }
    if (name.startsWith("~/")) {
      name = "~" + name.substring(2);
    }
    return name;
  }

  /**
   * Is this a /global/name?
   * 
   * <ul>
   * <li>
   * If node node1 in the global / namespace accesses the resource /bar, that
   * will resolve to the name /bar.</li>
   * <li>
   * If node node2 in the /wg/ namespace accesses the resource /foo, that will
   * resolve to the name /foo.</li>
   * <li>
   * If node node3 in the /wg/ namespace accesses the resource /foo/bar, that
   * will resolve to the name /foo/bar.</li>
   * </ul>
   * 
   * @return {@code true} if this name is a global name, {@code false} otherwise
   */
  public boolean isGlobal() {
    return !isEmpty() && name.charAt(0) == '/';
  }

  /**
   * @return {@code true} if this {@link GraphName} represents the root
   *         namespace, {@code false} otherwise
   */
  public boolean isRoot() {
    return name.equals(GraphName.ROOT);
  }

  /**
   * @return {@code true} if this {@link GraphName} is empty, {@code false}
   *         otherwise
   */
  public boolean isEmpty() {
    return name.isEmpty();
  }

  /**
   * Is this a ~private/name?
   * 
   * <ul>
   * <li>
   * If node node1 in the global / namespace accesses the resource ~bar, that
   * will resolve to the name /node1/bar.
   * <li>
   * If node node2 in the /wg/ namespace accesses the resource ~foo, that will
   * resolve to the name /wg/node2/foo.
   * <li>If node node3 in the /wg/ namespace accesses the resource ~foo/bar,
   * that will resolve to the name /wg/node3/foo/bar.
   * </ul>
   * 
   * @return {@code true} if the name is a private name, {@code false} otherwise
   */
  public boolean isPrivate() {
    return !isEmpty() && name.charAt(0) == '~';
  }

  /**
   * Is this a relative/name?
   * 
   * <ul>
   * <li>If node node1 in the global / namespace accesses the resource ~bar,
   * that will resolve to the name /node1/bar.
   * <li>If node node2 in the /wg/ namespace accesses the resource ~foo, that
   * will resolve to the name /wg/node2/foo.
   * <li>If node node3 in the /wg/ namespace accesses the resource ~foo/bar,
   * that will resolve to the name /wg/node3/foo/bar.
   * </ul>
   * 
   * @return true if the name is a relative name.
   */
  public boolean isRelative() {
    return !isPrivate() && !isGlobal();
  }

  /**
   * @return the parent of this {@link GraphName} in its canonical
   *         representation or an empty {@link GraphName} if there is no parent
   */
  public GraphName getParent() {
    if (name.length() == 0) {
      return GraphName.empty();
    }
    if (name.equals(GraphName.ROOT)) {
      return GraphName.root();
    }
    int slashIdx = name.lastIndexOf('/');
    if (slashIdx > 1) {
      return new GraphName(name.substring(0, slashIdx));
    }
    if (isGlobal()) {
      return GraphName.root();
    }
    return GraphName.empty();
  }

  /**
   * @return a {@link GraphName} without the leading parent namespace
   */
  public GraphName getBasename() {
    int slashIdx = name.lastIndexOf('/');
    if (slashIdx > -1) {
      if (slashIdx + 1 < name.length()) {
        return new GraphName(name.substring(slashIdx + 1));
      }
      return GraphName.empty();
    }
    return this;
  }

  /**
   * Convert name to a relative name representation. This does not take any
   * namespace into account; it simply strips any preceding characters for
   * global or private name representation.
   * 
   * @return a relative {@link GraphName}
   */
  public GraphName toRelative() {
    if (isPrivate() || isGlobal()) {
      return new GraphName(name.substring(1));
    }
    return this;
  }

  /**
   * Convert name to a global name representation. This does not take any
   * namespace into account; it simply adds in the global prefix "/" if missing.
   * 
   * @return a global {@link GraphName}
   */
  public GraphName toGlobal() {
    if (isGlobal()) {
      return this;
    }
    if (isPrivate()) {
      return new GraphName(GraphName.ROOT + name.substring(1));
    }
    return new GraphName(GraphName.ROOT + name);
  }

  /**
   * Join this {@link GraphName} with another.
   * 
   * @param other
   *          the {@link GraphName} to join with, if other is global, this will
   *          return other
   * @return a {@link GraphName} representing the concatenation of this
   *         {@link GraphName} and {@code other}
   */
  public GraphName join(GraphName other) {
    if (other.isGlobal() || isEmpty()) {
      return other;
    }
    if (isRoot()) {
      return other.toGlobal();
    }
    if (other.isEmpty()) {
      return this;
    }
    return new GraphName(toString() + SEPARATOR + other.toString());
  }

  /**
   * @see #join(GraphName)
   */
  public GraphName join(String other) {
    return join(GraphName.of(other));
  }

  @Override
  public String toString() {
    return name;
  }

  @Override
  public int hashCode() {
    return name.hashCode();
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    GraphName other = (GraphName) obj;
    if (name == null) {
      if (other.name != null)
        return false;
    } else if (!name.equals(other.name))
      return false;
    return true;
  }
}
