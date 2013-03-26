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

import static org.ros.Assert.assertGraphNameEquals;

import java.util.HashMap;
import java.util.Map;

import org.junit.Test;

/**
 * @author kwc@willowgarage.com (Ken Conley)
 */
public class NodeNameResolverTest {

  @Test
  public void testResolveNameOneArg() {
    Map<GraphName, GraphName> remappings = new HashMap<GraphName, GraphName>();
    GraphName nodeName = GraphName.of("/node");
    NodeNameResolver r = new NodeNameResolver(nodeName, NameResolver.newRootFromRemappings(remappings));

    assertGraphNameEquals("/foo", r.resolve("foo"));
    assertGraphNameEquals("/foo", r.resolve("/foo"));
    assertGraphNameEquals("/foo/bar", r.resolve("foo/bar"));

    assertGraphNameEquals("/node/foo", r.resolve("~foo"));
    assertGraphNameEquals("/node/foo/bar", r.resolve("~foo/bar"));
    // https://code.ros.org/trac/ros/ticket/3044
    assertGraphNameEquals("/node/foo", r.resolve("~/foo"));

    nodeName = GraphName.of("/ns1/node");
    r = new NodeNameResolver(nodeName, NameResolver.newRootFromRemappings(remappings));
    assertGraphNameEquals("/ns1/node/foo", r.resolve("~foo"));
    assertGraphNameEquals("/ns1/node/foo", r.resolve("~/foo"));
    assertGraphNameEquals("/ns1/node/foo/bar", r.resolve("~/foo/bar"));

    // Test case where private name is not is same namespace as default
    nodeName = GraphName.of("/ns2/node");
    r = new NodeNameResolver(nodeName, NameResolver.newFromNamespaceAndRemappings("/ns1", remappings));

    assertGraphNameEquals("/ns1/foo", r.resolve("foo"));
    assertGraphNameEquals("/foo", r.resolve("/foo"));
    assertGraphNameEquals("/ns1/foo/bar", r.resolve("foo/bar"));

    assertGraphNameEquals("/ns2/node/foo", r.resolve("~foo"));
    assertGraphNameEquals("/ns2/node/foo/bar", r.resolve("~foo/bar"));
    // https://code.ros.org/trac/ros/ticket/3044
    assertGraphNameEquals("/ns2/node/foo", r.resolve("~/foo"));
  }
}
