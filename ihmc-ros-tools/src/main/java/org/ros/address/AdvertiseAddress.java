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

import com.google.common.base.Preconditions;

import org.ros.exception.RosRuntimeException;

import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.URI;
import java.util.concurrent.Callable;

/**
 * A wrapper for {@link InetSocketAddress} that emphasizes the difference
 * between an address that should be used for binding a server port and one that
 * should be advertised to external entities.
 * 
 * An {@link AdvertiseAddress} encourages lazy lookups of port information to
 * prevent accidentally storing a bind port (e.g. 0 for OS picked) instead of
 * the advertised port.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class AdvertiseAddress {

  private final String host;

  private Callable<Integer> portCallable;

  public static AdvertiseAddress newPrivate() {
    return new PrivateAdvertiseAddressFactory().newDefault();
  }

  /**
   * Best effort method, returns a new {@link AdvertiseAddress} where the host
   * is determined automatically.
   * 
   * @return a suitable {@link AdvertiseAddress} for a publicly accessible
   *         {@link BindAddress}
   */
  public static AdvertiseAddress newPublic() {
    return new PublicAdvertiseAddressFactory().newDefault();
  }

  public AdvertiseAddress(String host) {
    Preconditions.checkNotNull(host);
    this.host = host;
  }

  public String getHost() {
    return host;
  }

  public void setStaticPort(final int port) {
    portCallable = new Callable<Integer>() {
      @Override
      public Integer call() throws Exception {
        return port;
      }
    };
  }

  public int getPort() {
    try {
      return portCallable.call();
    } catch (Exception e) {
      throw new RosRuntimeException(e);
    }
  }

  public void setPortCallable(Callable<Integer> portCallable) {
    this.portCallable = portCallable;
  }

  public InetAddress toInetAddress() {
    return InetAddressFactory.newFromHostString(host);
  }

  public InetSocketAddress toInetSocketAddress() {
    Preconditions.checkNotNull(portCallable);
    try {
      InetAddress address = toInetAddress();
      return new InetSocketAddress(address, portCallable.call());
    } catch (Exception e) {
      throw new RosRuntimeException(e);
    }
  }

  public URI toUri(String scheme) {
    Preconditions.checkNotNull(portCallable);
    try {
      return new URI(scheme, null, host, portCallable.call(), "/", null, null);
    } catch (Exception e) {
      throw new RosRuntimeException("Failed to create URI: " + this, e);
    }
  }

  public boolean isLoopbackAddress() {
    return toInetAddress().isLoopbackAddress();
  }

  @Override
  public String toString() {
    Preconditions.checkNotNull(portCallable);
    try {
      return "AdvertiseAddress<" + host + ", " + portCallable.call() + ">";
    } catch (Exception e) {
      throw new RosRuntimeException(e);
    }
  }

  @Override
  public int hashCode() {
    Preconditions.checkNotNull(portCallable);
    final int prime = 31;
    int result = 1;
    result = prime * result + ((host == null) ? 0 : host.hashCode());
    try {
      result = prime * result + portCallable.call();
    } catch (Exception e) {
      throw new RosRuntimeException(e);
    }
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    Preconditions.checkNotNull(portCallable);
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    AdvertiseAddress other = (AdvertiseAddress) obj;
    if (host == null) {
      if (other.host != null)
        return false;
    } else if (!host.equals(other.host))
      return false;
    try {
      if (portCallable.call() != other.portCallable.call())
        return false;
    } catch (Exception e) {
      throw new RosRuntimeException(e);
    }
    return true;
  }

}
