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

package org.ros.internal.message;

import org.junit.Before;
import org.junit.Test;
import org.ros.internal.message.service.ServiceDefinitionResourceProvider;
import org.ros.internal.message.service.ServiceRequestMessageFactory;
import org.ros.internal.message.service.ServiceResponseMessageFactory;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceTest {

  private ServiceDefinitionResourceProvider serviceDefinitionResourceProvider;
  private ServiceRequestMessageFactory serviceRequestMessageFactory;
  private ServiceResponseMessageFactory serviceResponseMessageFactory;

  @Before
  public void setUp() {
    serviceDefinitionResourceProvider = new ServiceDefinitionResourceProvider();
    serviceDefinitionResourceProvider.add("foo/Echo", "string data\n---\nstring data");
    serviceRequestMessageFactory =
        new ServiceRequestMessageFactory(serviceDefinitionResourceProvider);
    serviceResponseMessageFactory =
        new ServiceResponseMessageFactory(serviceDefinitionResourceProvider);
  }

  @Test
  public void testCreateEchoService() {
    RawMessage request = serviceRequestMessageFactory.newFromType("foo/Echo");
    RawMessage response = serviceResponseMessageFactory.newFromType("foo/Echo");
    request.setString("data", "Hello, ROS!");
    response.setString("data", "Hello, ROS!");
  }
}
