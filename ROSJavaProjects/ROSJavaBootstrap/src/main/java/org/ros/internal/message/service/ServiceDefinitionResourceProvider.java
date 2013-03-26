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

import com.google.common.base.Preconditions;

import org.ros.internal.message.StringResourceProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageIdentifier;

import java.util.Collection;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceDefinitionResourceProvider implements MessageDefinitionProvider {

  private final StringResourceProvider stringResourceProvider;

  public ServiceDefinitionResourceProvider() {
    stringResourceProvider = new StringResourceProvider();
  }

  private String serviceTypeToResourceName(String serviceType) {
    Preconditions.checkArgument(serviceType.contains("/"), "Service type must be fully qualified: "
        + serviceType);
    String[] packageAndType = serviceType.split("/", 2);
    return String.format("/%s/srv/%s.srv", packageAndType[0], packageAndType[1]);
  }

  @Override
  public String get(String serviceType) {
    return stringResourceProvider.get(serviceTypeToResourceName(serviceType));
  }

  @Override
  public boolean has(String serviceType) {
    return stringResourceProvider.has(serviceTypeToResourceName(serviceType));
  }

  public void add(String serviceType, String serviceDefinition) {
    stringResourceProvider.addStringToCache(serviceTypeToResourceName(serviceType),
        serviceDefinition);
  }

  @Override
  public Collection<String> getPackages() {
    throw new UnsupportedOperationException();
  }

  @Override
  public Collection<MessageIdentifier> getMessageIdentifiersByPackage(String pkg) {
    throw new UnsupportedOperationException();
  }
}
