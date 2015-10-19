/*
 * Copyright (C) 2012 Google Inc.
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

package org.ros.master.client;

import java.util.Collection;

/**
 * The state of the ROS graph as understood by the master.
 * 
 * @author Keith M. Hughes
 */
public class SystemState {

	/**
	 * All topics known.
	 */
	private final Collection<TopicSystemState> topics;

	public SystemState(Collection<TopicSystemState> topics) {
		this.topics = topics;
	}

	/**
	 * Get all topics in the system state.
	 * 
	 * @return a collection of topics.
	 */
	public Collection<TopicSystemState> getTopics() {
		return topics;
	}
}
