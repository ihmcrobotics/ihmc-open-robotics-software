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

package org.ros.internal.node.response;

import java.util.List;

import org.ros.master.client.TopicType;

import com.google.common.collect.Lists;

/**
 * A {@link ResultFactory} to take an object and turn it into a list of
 * {@link TopicType} instances.
 * 
 * @author Keith M. Hughes
 */
public class TopicTypeListResultFactory implements
		ResultFactory<List<TopicType>> {

	@Override
	public List<TopicType> newFromValue(Object value) {
		List<TopicType> topics = Lists.newArrayList();

		for (Object pair : (Object[]) value) {
			topics.add(new TopicType((String) ((Object[]) pair)[0],
					(String) ((Object[]) pair)[1]));
		}

		return topics;
	}

}
