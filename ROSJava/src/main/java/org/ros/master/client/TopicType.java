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

/**
 * A simple collection of information about a topic.
 *
 * @author Keith M. Hughes
 */
public class TopicType {
	
	/**
	 * Name of the topic.
	 */
	private final String name;
	
	/**
	 * Message type of the topic.
	 */
	private final String messageType;

	public TopicType(String name, String messageType) {
		this.name = name;
		this.messageType = messageType;
	}

	/**
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	/**
	 * @return the messageType
	 */
	public String getMessageType() {
		return messageType;
	}
}
