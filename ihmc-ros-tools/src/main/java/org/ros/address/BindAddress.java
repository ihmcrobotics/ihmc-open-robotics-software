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

package org.ros.address;

import java.net.InetSocketAddress;

/**
 * A wrapper for {@link InetSocketAddress} that emphasizes the difference
 * between an address that should be used for binding a server port and one that
 * should be advertised to external entities.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 */
public class BindAddress {

  private final InetSocketAddress address;

  private BindAddress(InetSocketAddress address) {
    this.address = address;
  }

  /**
   * @param port the port to bind to
   * @return a {@link BindAddress} instance with specified port that will bind
   *         to all network interfaces on the host
   */
  public static BindAddress newPublic(int port) {
    return new BindAddress(new InetSocketAddress(port));
  }

  public static BindAddress newPublic() {
    return newPublic(0);
  }

  /**
   * @param port the port to bind to
   * @return a {@link BindAddress} instance with specified port that will bind
   *         to the loopback interface on the host
   */
  public static BindAddress newPrivate(int port) {
    return new BindAddress(new InetSocketAddress(InetAddressFactory.newLoopback(), port));
  }

  public static BindAddress newPrivate() {
    return newPrivate(0);
  }

  @Override
  public String toString() {
    return "BindAddress<" + address + ">";
  }

  public InetSocketAddress toInetSocketAddress() {
    return address;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((address == null) ? 0 : address.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) return true;
    if (obj == null) return false;
    if (getClass() != obj.getClass()) return false;
    BindAddress other = (BindAddress) obj;
    if (address == null) {
      if (other.address != null) return false;
    } else if (!address.equals(other.address)) return false;
    return true;
  }

}
