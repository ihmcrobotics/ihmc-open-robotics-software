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

import org.ros.namespace.GraphName;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * Encapsulates a {@link Node} with its associated program logic.
 * 
 * <p>
 * {@link NodeMain} is the one required {@link NodeListener} for {@link Node}
 * creation. {@link NodeListener#onStart(ConnectedNode)} should be used to set up your
 * program's {@link Publisher}s, {@link Subscriber}s, etc.
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface NodeMain extends NodeListener {

  /**
   * @return the name of the {@link Node} that will be used if a name was not
   *         specified in the {@link Node}'s associated
   *         {@link NodeConfiguration}
   */
  GraphName getDefaultNodeName();
}
