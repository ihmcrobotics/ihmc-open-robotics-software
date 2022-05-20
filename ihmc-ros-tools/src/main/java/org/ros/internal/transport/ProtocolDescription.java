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

package org.ros.internal.transport;

import java.net.InetSocketAddress;
import java.util.List;

import org.ros.address.AdvertiseAddress;

import com.google.common.collect.Lists;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ProtocolDescription {

  private final String name;
  private final AdvertiseAddress address;

  public ProtocolDescription(String name, AdvertiseAddress address) {
    this.name = name;
    this.address = address;
  }

  public String getName() {
    return name;
  }

  public AdvertiseAddress getAdverstiseAddress() {
    return address;
  }
  
  public InetSocketAddress getAddress() {
    return address.toInetSocketAddress();
  }

  public List<Object> toList() {
    return Lists.newArrayList((Object) name, address.getHost(), address.getPort());
  }

  @Override
  public String toString() {
    return "Protocol<" + name + ", " + getAdverstiseAddress() + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((address == null) ? 0 : address.hashCode());
    result = prime * result + ((name == null) ? 0 : name.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    ProtocolDescription other = (ProtocolDescription) obj;
    if (address == null) {
      if (other.address != null)
        return false;
    } else if (!address.equals(other.address))
      return false;
    if (name == null) {
      if (other.name != null)
        return false;
    } else if (!name.equals(other.name))
      return false;
    return true;
  }

}
