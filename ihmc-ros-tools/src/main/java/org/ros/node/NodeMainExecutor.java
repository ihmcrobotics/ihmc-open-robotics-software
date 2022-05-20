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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.ScheduledExecutorService;

/**
 * Executes {@link NodeMain}s and allows shutting down individual
 * {@link NodeMain}s or all currently running {@link NodeMain}s as a group.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface NodeMainExecutor {

  /**
   * @return the {@link ScheduledExecutorService} that this
   *         {@link NodeMainExecutor} uses
   */
  ScheduledExecutorService getScheduledExecutorService();

  /**
   * Executes the supplied {@link NodeMain} using the supplied
   * {@link NodeConfiguration}.
   * 
   * @param nodeMain
   *          the {@link NodeMain} to execute
   * @param nodeConfiguration
   *          the {@link NodeConfiguration} that will be used to create the
   *          {@link Node}
   * @param nodeListeners
   *          a {@link Collection} of {@link NodeListener}s to be added to the
   *          {@link Node} before it starts, can be {@code null}
   */
  void execute(NodeMain nodeMain, NodeConfiguration nodeConfiguration,
      Collection<NodeListener> nodeListeners);

  /**
   * Executes the supplied {@link NodeMain} using the supplied
   * {@link NodeConfiguration}.
   * 
   * @param nodeMain
   *          the {@link NodeMain} to execute
   * @param nodeConfiguration
   *          the {@link NodeConfiguration} that will be used to create the
   *          {@link Node}
   */
  void execute(NodeMain nodeMain, NodeConfiguration nodeConfiguration);

  /**
   * Shuts down the supplied {@link NodeMain} (i.e.
   * {@link NodeMain#onShutdown(Node)} will be called). This does not
   * necessarily shut down the {@link Node} that is associated with the
   * {@link NodeMain}.
   * 
   * <p>
   * This has no effect if the {@link NodeMain} has not started.
   * 
   * @param nodeMain
   *          the {@link NodeMain} to shutdown
   */
  void shutdownNodeMain(NodeMain nodeMain);

  /**
   * Shutdown all started {@link Node}s. This does not shut down the supplied
   * {@link ExecutorService}.
   */
  void shutdown();
}
