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

package org.ros.node.parameter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import org.junit.Before;
import org.junit.Test;
import org.ros.RosTest;
import org.ros.exception.ParameterClassCastException;
import org.ros.exception.ParameterNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ParameterTreeIntegrationTest extends RosTest {

  private ParameterTree parameters;

  @Before
  public void setup() throws InterruptedException {
    final CountDownLatch latch = new CountDownLatch(1);
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("node_name");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        parameters = connectedNode.getParameterTree();
        latch.countDown();
      }
    }, nodeConfiguration);
    assertTrue(latch.await(1, TimeUnit.SECONDS));
  }

  @Test
  public void testGetNonExistentParameter() {
    try {
      parameters.getBoolean("bloop");
      fail();
    } catch (ParameterNotFoundException e) {
      // Thrown when a parameter does not exist.
    }
  }

  @Test
  public void testGetParameterOfWrongType() {
    parameters.set("bloop", "foo");
    try {
      parameters.getBoolean("bloop");
      fail();
    } catch (ParameterClassCastException e) {
      // Thrown when a parameter is of the wrong type.
    }
  }

  @Test
  public void testGetParameterWithDefault() {
    assertTrue(parameters.getBoolean("bloop", true));
    List<String> expectedList = Lists.newArrayList("foo", "bar", "baz");
    assertEquals(expectedList, parameters.getList("bloop", expectedList));
    parameters.set("bloop", expectedList);
    assertEquals(expectedList, parameters.getList("bloop", Lists.newArrayList()));
  }

  @Test
  public void testGetParameterWithDefaultOfWrongType() {
    parameters.set("bloop", "foo");
    try {
      parameters.getBoolean("bloop", true);
      fail();
    } catch (ParameterClassCastException e) {
      // Thrown when a parameter is of the wrong type.
    }
  }

  @Test
  public void testSetAndGetStrings() {
    parameters.set("/foo/bar", "baz");
    assertEquals("baz", parameters.getString("/foo/bar"));
    parameters.set("/foo/bar", "baz");
    assertEquals("baz", parameters.getString("/foo/bar"));
    Map<String, Object> expected = Maps.newHashMap();
    expected.put("bar", "baz");
    assertEquals(expected, parameters.getMap("/foo"));
  }

  @Test
  public void testSetAndGetAllTypes() {
    String name = "/foo/bar";
    parameters.set(name, true);
    assertTrue(parameters.getBoolean(name));
    parameters.set(name, 42);
    assertEquals(42, parameters.getInteger(name));
    parameters.set(name, 0.42d);
    assertEquals(0.42d, parameters.getDouble(name), 0.01);
    parameters.set(name, "foo");
    assertEquals("foo", parameters.getString(name));
    List<String> expectedList = Lists.newArrayList("foo", "bar", "baz");
    parameters.set(name, expectedList);
    assertEquals(expectedList, parameters.getList(name));
    Map<String, String> expectedMap = Maps.newHashMap();
    expectedMap.put("foo", "bar");
    expectedMap.put("baz", "bloop");
    parameters.set(name, expectedMap);
    assertEquals(expectedMap, parameters.getMap(name));
  }

  @Test
  public void testDeleteAndHas() {
    parameters.set("/foo/bar", "baz");
    assertTrue(parameters.has("/foo/bar"));
    parameters.delete("/foo/bar");
    assertFalse(parameters.has("/foo/bar"));
  }

  @Test
  public void testGetNames() {
    parameters.set("/foo/bar", "baz");
    parameters.set("/bloop", "doh");
    Collection<GraphName> names = parameters.getNames();
    assertEquals(2, names.size());
    assertTrue(names.contains(GraphName.of("/foo/bar")));
    assertTrue(names.contains(GraphName.of("/bloop")));
  }

  @Test
  public void testParameterPubSub() throws InterruptedException {
    final CountDownLatch nodeLatch = new CountDownLatch(1);
    final CountDownLatch parameterLatch = new CountDownLatch(1);
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("subscriber");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        ParameterTree subscriberParameters = connectedNode.getParameterTree();
        subscriberParameters.addParameterListener("/foo/bar", new ParameterListener() {
          @Override
          public void onNewValue(Object value) {
            assertEquals(42, value);
            parameterLatch.countDown();
          }
        });
        nodeLatch.countDown();
      }
    }, nodeConfiguration);
    nodeLatch.await(1, TimeUnit.SECONDS);
    parameters.set("/foo/bar", 42);
    assertTrue(parameterLatch.await(1, TimeUnit.SECONDS));
  }
}
