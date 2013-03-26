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

package org.ros.internal.loader;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.ros.Assert.assertGraphNameEquals;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.ros.CommandLineVariables;
import org.ros.EnvironmentVariables;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Test CommandLineLoader.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 */
public class CommandLineLoaderTest {

  private URI defaultMasterUri;
  private File defaultRosRoot;
  private List<String> emptyArgv;

  private HashMap<String, String> getDefaultEnv() {
    HashMap<String, String> env = new HashMap<String, String>();
    env.put(EnvironmentVariables.ROS_MASTER_URI, defaultMasterUri.toString());
    env.put(EnvironmentVariables.ROS_ROOT, defaultRosRoot.getAbsolutePath());
    return env;
  }

  @Before
  public void setup() throws URISyntaxException {
    defaultMasterUri = new URI("http://localhost:33133");
    defaultRosRoot = new File(System.getProperty("user.dir"));
    emptyArgv = Lists.newArrayList("Foo");
  }

  @Test
  public void testCommandLineLoader() {
    // Test with no args.
    CommandLineLoader loader = new CommandLineLoader(emptyArgv);
    assertEquals(0, loader.getNodeArguments().size());

    // Test with no remappings.
    emptyArgv.add("one two --three");
    loader = new CommandLineLoader(emptyArgv);
    Assert.assertEquals(emptyArgv.subList(1, emptyArgv.size()), loader.getNodeArguments());

    // Test with actual remappings. All of these are equivalent.
    List<String> tests =
        Lists.newArrayList("--help one name:=/my/name -i foo:=/my/foo",
            "name:=/my/name --help one -i foo:=/my/foo",
            "name:=/my/name --help foo:=/my/foo one -i");
    List<String> expected = Lists.newArrayList("--help", "one", "-i");
    for (String test : tests) {
      loader = new CommandLineLoader(Lists.newArrayList(("Foo " + test).split("\\s+")));
      // Test command-line parsing
      Assert.assertEquals(expected, loader.getNodeArguments());
    }
  }

  /**
   * Test createConfiguration() with respect to reading of the environment
   * configuration, including command-line overrides.
   */
  @Test
  public void testcreateConfigurationEnvironment() {
    // Construct artificial environment. Test failure without required settings.
    // Failure: ROS_ROOT not set.
    String tmpDir = System.getProperty("java.io.tmpdir");
    String rosPackagePath = tmpDir + File.pathSeparator + defaultRosRoot;
    List<File> rosPackagePathList = Lists.newArrayList(new File(tmpDir), defaultRosRoot);
    Map<String, String> env = new HashMap<String, String>();
    CommandLineLoader loader = null;
    env = Maps.newHashMap();
    env.put(EnvironmentVariables.ROS_ROOT, defaultRosRoot.getAbsolutePath());
    loader = new CommandLineLoader(emptyArgv, env);
    NodeConfiguration nodeConfiguration = loader.build();
    assertEquals(NodeConfiguration.DEFAULT_MASTER_URI, nodeConfiguration.getMasterUri());

    // Construct artificial environment. Set required environment variables.
    env = getDefaultEnv();
    loader = new CommandLineLoader(emptyArgv, env);
    nodeConfiguration = loader.build();
    assertEquals(defaultMasterUri, nodeConfiguration.getMasterUri());
    assertEquals(defaultRosRoot, nodeConfiguration.getRosRoot());
    assertTrue(nodeConfiguration.getParentResolver().getNamespace().isRoot());
    // Default is loopback.
    assertTrue(nodeConfiguration.getTcpRosAdvertiseAddress().isLoopbackAddress());
    assertTrue(nodeConfiguration.getXmlRpcAdvertiseAddress().isLoopbackAddress());

    // Construct artificial environment. Set optional environment variables.
    env = getDefaultEnv();
    env.put(EnvironmentVariables.ROS_IP, "192.168.0.1");
    env.put(EnvironmentVariables.ROS_NAMESPACE, "/foo/bar");
    env.put(EnvironmentVariables.ROS_PACKAGE_PATH, rosPackagePath);
    loader = new CommandLineLoader(emptyArgv, env);
    nodeConfiguration = loader.build();

    assertEquals(defaultMasterUri, nodeConfiguration.getMasterUri());
    assertEquals(defaultRosRoot, nodeConfiguration.getRosRoot());
    assertEquals("192.168.0.1", nodeConfiguration.getTcpRosAdvertiseAddress().getHost());
    assertEquals("192.168.0.1", nodeConfiguration.getXmlRpcAdvertiseAddress().getHost());
    assertEquals(GraphName.of("/foo/bar"), nodeConfiguration.getParentResolver().getNamespace());
    Assert.assertEquals(rosPackagePathList, nodeConfiguration.getRosPackagePath());

    // Test ROS namespace resolution and canonicalization
    GraphName canonical = GraphName.of("/baz/bar");
    env = getDefaultEnv();
    env.put(EnvironmentVariables.ROS_NAMESPACE, "baz/bar");
    loader = new CommandLineLoader(emptyArgv, env);
    nodeConfiguration = loader.build();
    assertEquals(canonical, nodeConfiguration.getParentResolver().getNamespace());
    env = getDefaultEnv();
    env.put(EnvironmentVariables.ROS_NAMESPACE, "baz/bar/");
    loader = new CommandLineLoader(emptyArgv, env);
    nodeConfiguration = loader.build();
    assertEquals(canonical, nodeConfiguration.getParentResolver().getNamespace());
  }

  @Test
  public void testCreateConfigurationCommandLine() throws URISyntaxException {
    Map<String, String> env = getDefaultEnv();

    // Test __name override
    NodeConfiguration nodeConfiguration = new CommandLineLoader(emptyArgv, env).build();
    assertEquals(null, nodeConfiguration.getNodeName());
    List<String> args = Lists.newArrayList("Foo", "__name:=newname");
    nodeConfiguration = new CommandLineLoader(args, env).build();
    assertEquals("newname", nodeConfiguration.getNodeName().toString());

    // Test ROS_MASTER_URI from command-line
    args =
        Lists.newArrayList("Foo", CommandLineVariables.ROS_MASTER_URI + ":=http://override:22622");
    nodeConfiguration = new CommandLineLoader(args, env).build();
    assertEquals(new URI("http://override:22622"), nodeConfiguration.getMasterUri());

    // Test again with env var removed, make sure that it still behaves the
    // same.
    env.remove(EnvironmentVariables.ROS_MASTER_URI);
    nodeConfiguration = new CommandLineLoader(args, env).build();
    assertEquals(new URI("http://override:22622"), nodeConfiguration.getMasterUri());

    // Test ROS namespace resolution and canonicalization
    GraphName canonical = GraphName.of("/baz/bar");
    env = getDefaultEnv();
    args = Lists.newArrayList("Foo", CommandLineVariables.ROS_NAMESPACE + ":=baz/bar");
    nodeConfiguration = new CommandLineLoader(args, env).build();
    assertEquals(canonical, nodeConfiguration.getParentResolver().getNamespace());

    args = Lists.newArrayList("Foo", CommandLineVariables.ROS_NAMESPACE + ":=baz/bar/");
    nodeConfiguration = new CommandLineLoader(args, env).build();
    assertEquals(canonical, nodeConfiguration.getParentResolver().getNamespace());

    // Verify precedence of command-line __ns over environment.
    env.put(EnvironmentVariables.ROS_NAMESPACE, "wrong/answer/");
    nodeConfiguration = new CommandLineLoader(args, env).build();
    assertEquals(canonical, nodeConfiguration.getParentResolver().getNamespace());

    // Verify address override.
    env = getDefaultEnv();
    args = Lists.newArrayList("Foo", CommandLineVariables.ROS_IP + ":=192.168.0.2");
    nodeConfiguration = new CommandLineLoader(args, env).build();
    assertEquals("192.168.0.2", nodeConfiguration.getTcpRosAdvertiseAddress().getHost());
    assertEquals("192.168.0.2", nodeConfiguration.getXmlRpcAdvertiseAddress().getHost());

    // Verify multiple options work together.
    env = getDefaultEnv();
    args =
        Lists.newArrayList("Foo", CommandLineVariables.ROS_NAMESPACE + ":=baz/bar/", "ignore",
            CommandLineVariables.ROS_MASTER_URI + ":=http://override:22622", "--bad",
            CommandLineVariables.ROS_IP + ":=192.168.0.2");
    nodeConfiguration = new CommandLineLoader(args, env).build();
    assertEquals(new URI("http://override:22622"), nodeConfiguration.getMasterUri());
    assertEquals("192.168.0.2", nodeConfiguration.getTcpRosAdvertiseAddress().getHost());
    assertEquals("192.168.0.2", nodeConfiguration.getXmlRpcAdvertiseAddress().getHost());
    assertEquals(canonical, nodeConfiguration.getParentResolver().getNamespace());
  }

  /**
   * Test the {@link NameResolver} created by createConfiguration().
   */
  @Test
  public void testcreateConfigurationResolver() {
    // Construct artificial environment. Set required environment variables.
    HashMap<String, String> env = getDefaultEnv();

    // Test with no args.
    CommandLineLoader loader = new CommandLineLoader(emptyArgv, env);
    NodeConfiguration nodeConfiguration = loader.build();
    nodeConfiguration.getParentResolver().getRemappings();
    nodeConfiguration = loader.build();

    // Test with no remappings.
    List<String> args = Lists.newArrayList("Foo", "foo", "--bar");
    loader = new CommandLineLoader(args, env);
    nodeConfiguration = loader.build();
    NameResolver resolver = nodeConfiguration.getParentResolver();
    assertTrue(resolver.getRemappings().isEmpty());

    // test with actual remappings. All of these tests are equivalent.
    List<String> tests =
        Lists.newArrayList("--help name:=/my/name -i foo:=/my/foo",
            "name:=/my/name --help -i foo:=/my/foo", "name:=/my/name foo:=/my/foo --help -i");
    for (String test : tests) {
      args = Lists.newArrayList(("Foo " + test).split("\\s+"));
      loader = new CommandLineLoader(args, env);
      nodeConfiguration = loader.build();

      // Test that our remappings loaded.
      NameResolver r = nodeConfiguration.getParentResolver();
      GraphName n = r.resolve("name");
      assertGraphNameEquals("/my/name", n);
      assertGraphNameEquals("/name", r.resolve("/name"));
      assertGraphNameEquals("/my/foo", r.resolve("foo"));
      assertGraphNameEquals("/my/name", r.resolve("/my/name"));
    }
  }
}
