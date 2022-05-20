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

package org.ros.master.uri;

import com.google.common.collect.Lists;

import org.ros.exception.RosRuntimeException;

import java.net.URI;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * A proxying {@link MasterUriProvider} which can be switched between providers.
 * 
 * <p>
 * This class is thread-safe.
 * 
 * @author Keith M. Hughes
 */
public class SwitchableMasterUriProvider implements MasterUriProvider {

  private final Object mutex;

  /**
   * The current provider in use.
   */
  private MasterUriProvider provider;

  /**
   * The list of all pending requests.
   */
  private List<ProviderRequest> pending = Lists.newArrayList();

  /**
   * @param provider
   *          the initial provider to use
   */
  public SwitchableMasterUriProvider(MasterUriProvider provider) {
    this.provider = provider;
    mutex = new Object();
  }

  @Override
  public URI getMasterUri() throws RosRuntimeException {
    MasterUriProvider providerToUse = null;
    ProviderRequest requestToUse = null;

    synchronized (mutex) {
      if (provider != null) {
        providerToUse = provider;
      } else {
        requestToUse = new ProviderRequest();
        pending.add(requestToUse);
      }
    }

    if (providerToUse != null) {
      return providerToUse.getMasterUri();
    } else {
      return requestToUse.getMasterUri();
    }
  }

  @Override
  public URI getMasterUri(long timeout, TimeUnit unit) {
    // We can't really switch providers, but people are willing to wait. It
    // seems appropriate to wait rather than to return immediately.

    MasterUriProvider providerToUse = null;
    synchronized (mutex) {
      if (provider != null) {
        providerToUse = provider;
      }
    }

    if (providerToUse != null) {
      return providerToUse.getMasterUri(timeout, unit);
    } else {
      try {
        Thread.sleep(unit.toMillis(timeout));
      } catch (InterruptedException e) {
        // Don't care
      }

      return null;
    }
  }

  /**
   * Switch between providers.
   * 
   * @param switcher
   *          the new provider
   */
  public void switchProvider(MasterUriProviderSwitcher switcher) {
    synchronized (mutex) {
      MasterUriProvider oldProvider = provider;
      provider = switcher.switchProvider(oldProvider);

      if (oldProvider == null) {
        for (ProviderRequest request : pending) {
          request.setProvider(provider);
        }
        pending.clear();
      }
    }
  }

  /**
   * Perform a switch between {@link MasterUriProvider} instances for the
   * {@link SwitchableMasterUriProvider}.
   * 
   * <p>
   * This class permits the use of atomic provider switches.
   */
  public interface MasterUriProviderSwitcher {

    /**
     * Switch the provider in use.
     * 
     * @param oldProvider
     *          a reference to the provider which came before
     * 
     * @return the new provider to use
     */
    MasterUriProvider switchProvider(MasterUriProvider oldProvider);
  }

  /**
   * A request for a URI which is blocked until it is available.
   */
  private static class ProviderRequest {

    /**
     * The latch used to wait.
     */
    private CountDownLatch latch = new CountDownLatch(1);

    /**
     * The provider which will give the eventual answer.
     */
    private MasterUriProvider provider;

    /**
     * Get a service.
     * 
     * <p>
     * This call can block indefinitely.
     * 
     * @return the master {@link URI}
     */
    public URI getMasterUri() {
      try {
        latch.await();
        return provider.getMasterUri();
      } catch (InterruptedException e) {
        throw new RosRuntimeException("URI provider interrupted", e);
      }
    }

    /**
     * Set the provider who will finally process the request.
     * 
     * @param provider
     *          the {@link MasterUriProvider} to use
     */
    public void setProvider(MasterUriProvider provider) {
      this.provider = provider;
      latch.countDown();
    }
  }
}
