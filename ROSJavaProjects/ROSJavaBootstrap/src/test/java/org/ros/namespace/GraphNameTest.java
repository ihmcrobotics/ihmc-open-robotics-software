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
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class GraphNameTest {

  public void assertGraphNameEquals(String name, GraphName graphName) {
    assertEquals(name, graphName.toString());
  }

  @Test
  public void testToString() {
    try {
      String[] canonical = { "abc", "ab7", "/abc", "/abc/bar", "/", "~garage", "~foo/bar" };
      for (String c : canonical) {
        assertGraphNameEquals(c, GraphName.of(c));
      }
    } catch (IllegalArgumentException e) {
      fail("These names should be valid" + e.toString());
    }
  }

  @Test
  public void testValidNames() {
    String[] valid =
        { "", "abc", "ab7", "ab7_kdfJKSDJFGkd", "/abc", "/", "~private", "~private/something",
            "/global", "/global/", "/global/local" };
    for (String v : valid) {
      GraphName.of(v);
    }
  }

  @Test
  public void testInvalidNames() {
    final String[] illegalChars = { "=", "-", "(", ")", "*", "%", "^" };
    for (String i : illegalChars) {
      try {
        GraphName.of("good" + i);
        fail("Bad name not caught: " + i);
      } catch (RuntimeException e) {
      }
    }
    final String[] illegalNames = { "/~private", "5foo" };
    for (String i : illegalNames) {
      try {
        GraphName.of(i);
        fail("Bad name not caught" + i);
      } catch (RuntimeException e) {
      }
    }
  }

  @Test
  public void testIsGlobal() {
    final String[] tests = { "/", "/global", "/global2" };
    for (String t : tests) {
      assertTrue(GraphName.of(t).isGlobal());
    }
    final String[] fails = { "", "not_global", "not/global" };
    for (String t : fails) {
      assertFalse(GraphName.of(t).isGlobal());
    }
  }

  @Test
  public void testIsPrivate() {
    String[] tests = { "~name", "~name/sub" };
    for (String t : tests) {
      assertTrue(GraphName.of(t).isPrivate());
    }
    String[] fails = { "", "not_private", "not/private", "/" };
    for (String f : fails) {
      assertFalse(GraphName.of(f).isPrivate());
    }
  }

  @Test
  public void testIsRelative() {
    GraphName n = GraphName.of("name");
    assertTrue(n.isRelative());
    n = GraphName.of("/name");
    assertFalse(n.isRelative());
  }

  @Test
  public void testGetParent() {
    GraphName global = GraphName.of("/");
    GraphName empty = GraphName.of("");
    // parent of empty is empty, just like dirname
    assertEquals(empty, GraphName.of("").getParent());
    // parent of global is global, just like dirname
    assertEquals(global, GraphName.of("/").getParent());

    // test with global names
    assertEquals(GraphName.of("/wg"), GraphName.of("/wg/name").getParent());
    assertEquals(GraphName.of("/wg"), GraphName.of("/wg/name/").getParent());
    assertEquals(global, GraphName.of("/wg/").getParent());
    assertEquals(global, GraphName.of("/wg").getParent());

    // test with relative names
    assertEquals(GraphName.of("wg"), GraphName.of("wg/name").getParent());
    assertEquals(empty, GraphName.of("wg/").getParent());
  }

  @Test
  public void testCanonicalizeName() {
    assertGraphNameEquals("", GraphName.of(""));
    assertGraphNameEquals("/", GraphName.of("/"));
    assertGraphNameEquals("/", GraphName.of("//"));
    assertGraphNameEquals("/", GraphName.of("///"));

    assertGraphNameEquals("foo", GraphName.of("foo"));
    assertGraphNameEquals("foo", GraphName.of("foo/"));
    assertGraphNameEquals("foo", GraphName.of("foo//"));

    assertGraphNameEquals("/foo", GraphName.of("/foo"));
    assertGraphNameEquals("/foo", GraphName.of("/foo/"));
    assertGraphNameEquals("/foo", GraphName.of("/foo//"));

    assertGraphNameEquals("/foo/bar", GraphName.of("/foo/bar"));
    assertGraphNameEquals("/foo/bar", GraphName.of("/foo/bar/"));
    assertGraphNameEquals("/foo/bar", GraphName.of("/foo/bar//"));

    assertGraphNameEquals("~foo", GraphName.of("~foo"));
    assertGraphNameEquals("~foo", GraphName.of("~foo/"));
    assertGraphNameEquals("~foo", GraphName.of("~foo//"));
    assertGraphNameEquals("~foo", GraphName.of("~/foo"));
  }

  @Test
  public void testGetName() {
    assertGraphNameEquals("", GraphName.of("").getBasename());
    assertGraphNameEquals("", GraphName.of("").getBasename());
    assertGraphNameEquals("foo", GraphName.of("/foo").getBasename());
    assertGraphNameEquals("foo", GraphName.of("foo").getBasename());
    // The trailing slash is removed when creating a GraphName.
    assertGraphNameEquals("foo", GraphName.of("foo/").getBasename());
    assertGraphNameEquals("bar", GraphName.of("/foo/bar").getBasename());
    assertGraphNameEquals("bar", GraphName.of("foo/bar").getBasename());
  }

  @Test
  public void testJoin() {
    assertEquals(GraphName.of("/bar"), GraphName.of("/").join(GraphName.of("bar")));
    assertEquals(GraphName.of("bar"), GraphName.of("").join(GraphName.of("bar")));
    assertEquals(GraphName.of("bar"), GraphName.of("bar").join(GraphName.of("")));
    assertEquals(GraphName.of("foo/bar"), GraphName.of("foo").join(GraphName.of("bar")));
    assertEquals(GraphName.of("/foo/bar"), GraphName.of("/foo").join(GraphName.of("bar")));
    assertEquals(GraphName.of("/bar"), GraphName.of("/foo").join(GraphName.of("/bar")));
  }

  @Test
  public void testNewAnonymous() throws InterruptedException {
    Executor executor = Executors.newFixedThreadPool(10);
    int sampleSize = 10000;
    final CountDownLatch latch = new CountDownLatch(sampleSize);
    final Set<GraphName> anonymousGraphNames =
        Collections.synchronizedSet(new HashSet<GraphName>());
    for (int i = 0; i < sampleSize; i++) {
      executor.execute(new Runnable() {
        @Override
        public void run() {
          GraphName name = GraphName.newAnonymous();
          assertTrue(name.toString().startsWith(GraphName.ANONYMOUS_PREFIX));
          anonymousGraphNames.add(name);
          latch.countDown();
        }
      });
    }
    assertTrue(latch.await(1, TimeUnit.SECONDS));
    assertEquals(sampleSize, anonymousGraphNames.size());
  }
}
