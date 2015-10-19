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

package org.ros.internal.message.service;

import org.ros.internal.message.Md5Generator;
import org.ros.message.MessageDefinitionProvider;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceDescriptionFactory {

  private final MessageDefinitionProvider messageDefinitionProvider;
  private final Md5Generator md5Generator;

  public ServiceDescriptionFactory(MessageDefinitionProvider messageDefinitionProvider) {
    this.messageDefinitionProvider = messageDefinitionProvider;
    md5Generator = new Md5Generator(messageDefinitionProvider);
  }

  public ServiceDescription newFromType(String serviceType) {
    String serviceDefinition = messageDefinitionProvider.get(serviceType);
    String md5Checksum = md5Generator.generate(serviceType);
    return new ServiceDescription(serviceType, serviceDefinition, md5Checksum);
  }

  public boolean hasType(String serviceType) {
    return messageDefinitionProvider.has(serviceType);
  }
}
