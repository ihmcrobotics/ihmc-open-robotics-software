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

package org.ros.internal.node.response;

import com.google.common.collect.Lists;

import org.ros.internal.message.topic.TopicDescription;


import org.ros.internal.node.topic.TopicDeclaration;
import org.ros.namespace.GraphName;

import java.util.Arrays;
import java.util.List;

/**
 * A {@link ResultFactory} to take an object and turn it into a list of
 * {@link TopicDeclaration} instances.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TopicListResultFactory implements ResultFactory<List<TopicDeclaration>> {

  @Override
  public List<TopicDeclaration> newFromValue(Object value) {
    List<TopicDeclaration> descriptions = Lists.newArrayList();
    List<Object> topics = Arrays.asList((Object[]) value);
    for (Object topic : topics) {
      String name = (String) ((Object[]) topic)[0];
      String type = (String) ((Object[]) topic)[1];
      descriptions.add(TopicDeclaration.newFromTopicName(GraphName.of(name), new TopicDescription(type, null,
          null)));
    }
    return descriptions;
  }
}
