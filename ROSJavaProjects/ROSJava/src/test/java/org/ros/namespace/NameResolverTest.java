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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;
import static org.ros.Assert.assertGraphNameEquals;

import java.util.HashMap;

import org.junit.Test;

/**
 * @author kwc@willowgarage.com (Ken Conley)
 */
public class NameResolverTest {

  @Test
  public void testResolveNameOneArg() {
    NameResolver r = NameResolver.newRoot();

    assertGraphNameEquals("/foo", r.resolve("foo"));
    assertGraphNameEquals("/foo", r.resolve("/foo"));
    assertGraphNameEquals("/foo/bar", r.resolve("foo/bar"));

    try {
      r.resolve("~foo");
      fail();
    } catch (RuntimeException e) {
      // Should throw if trying to resolve private name.
    }

    r = NameResolver.newFromNamespace("/ns1");

    assertGraphNameEquals("/ns1/foo", r.resolve("foo"));
    assertGraphNameEquals("/foo", r.resolve("/foo"));
    assertGraphNameEquals("/ns1/foo/bar", r.resolve("foo/bar"));
  }

  @Test
  public void testResolveNameTwoArg() {
    // These tests are based on test_roslib_names.py.
    NameResolver r = NameResolver.newRoot();
    try {
      r.resolve("foo", "bar");
      fail("should have raised");
    } catch (IllegalArgumentException e) {
    }

    GraphName root = GraphName.root();
    assertEquals(root, r.resolve(root, ""));
    assertEquals(root, r.resolve(root, root));
    assertEquals(root, r.resolve("/anything/bar", root));
    assertGraphNameEquals("/ns1/node", r.resolve("/ns1/node", ""));

    // relative namespaces get resolved to default namespace
    assertGraphNameEquals("/foo", r.resolve(root, "foo"));
    assertGraphNameEquals("/foo", r.resolve(root, "foo/"));
    assertGraphNameEquals("/foo", r.resolve(root, "/foo"));
    assertGraphNameEquals("/foo", r.resolve(root, "/foo/"));

    assertGraphNameEquals("/ns1/ns2/foo", r.resolve("/ns1/ns2", "foo"));
    assertGraphNameEquals("/ns1/ns2/foo", r.resolve("/ns1/ns2", "foo/"));
    assertGraphNameEquals("/ns1/ns2/foo", r.resolve("/ns1/ns2/", "foo"));
    assertGraphNameEquals("/foo", r.resolve("/ns1/ns2", "/foo/"));

    assertGraphNameEquals("/ns1/ns2/ns3/foo", r.resolve("/ns1/ns2/ns3", "foo"));
    assertGraphNameEquals("/ns1/ns2/ns3/foo", r.resolve("/ns1/ns2/ns3/", "foo"));
    assertGraphNameEquals("/foo", r.resolve(root, "/foo/"));

    assertGraphNameEquals("/ns1/ns2/foo/bar", r.resolve("/ns1/ns2", "foo/bar"));
    assertGraphNameEquals("/ns1/ns2/ns3/foo/bar", r.resolve("/ns1/ns2/ns3", "foo/bar"));

    try {
      r.resolve(root, "~foo");
      fail();
    } catch (RuntimeException e) {
      // resolveName() with two args should never allow private names
    }
  }

  /**
   * Test resolveName with name remapping active.
   */
  @Test
  public void testResolveNameRemapping() {
    HashMap<GraphName, GraphName> remappings = new HashMap<GraphName, GraphName>();
    remappings.put(GraphName.of("name"), GraphName.of("/my/name"));
    remappings.put(GraphName.of("foo"), GraphName.of("/my/foo"));

    NameResolver r = NameResolver.newRootFromRemappings(remappings);

    GraphName n = r.resolve("name");
    assertGraphNameEquals("/my/name", n);
    assertGraphNameEquals("/name", r.resolve("/name"));
    assertGraphNameEquals("/my/foo", r.resolve("foo"));
    assertGraphNameEquals("/my/name", r.resolve("/my/name"));
  }
}
