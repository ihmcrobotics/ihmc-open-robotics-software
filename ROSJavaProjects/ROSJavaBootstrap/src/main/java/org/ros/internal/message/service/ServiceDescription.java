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

import org.ros.internal.message.definition.MessageDefinitionTupleParser;

import org.ros.message.MessageDeclaration;
import org.ros.message.MessageIdentifier;

import java.util.List;

/**
 * The description of a ROS service.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceDescription extends MessageDeclaration {

  private final String requestType;
  private final String requestDefinition;
  private final String responseType;
  private final String responseDefinition;
  private final String md5Checksum;

  public ServiceDescription(String type, String definition, String md5Checksum) {
    super(MessageIdentifier.of(type), definition);
    this.md5Checksum = md5Checksum;
    List<String> requestAndResponse = MessageDefinitionTupleParser.parse(definition, 2);
    requestType = type + "Request";
    responseType = type + "Response";
    requestDefinition = requestAndResponse.get(0);
    responseDefinition = requestAndResponse.get(1);
  }

  public String getMd5Checksum() {
    return md5Checksum;
  }

  public String getRequestType() {
    return requestType;
  }

  public String getRequestDefinition() {
    return requestDefinition;
  }

  public String getResponseType() {
    return responseType;
  }

  public String getResponseDefinition() {
    return responseDefinition;
  }

  @Override
  public String toString() {
    return "ServiceDescription<" + getType() + ", " + md5Checksum + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result = prime * result + ((md5Checksum == null) ? 0 : md5Checksum.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (!super.equals(obj))
      return false;
    if (getClass() != obj.getClass())
      return false;
    ServiceDescription other = (ServiceDescription) obj;
    if (md5Checksum == null) {
      if (other.md5Checksum != null)
        return false;
    } else if (!md5Checksum.equals(other.md5Checksum))
      return false;
    return true;
  }
}
