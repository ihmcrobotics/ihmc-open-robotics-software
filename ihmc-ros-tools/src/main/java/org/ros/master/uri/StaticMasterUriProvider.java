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

package org.ros.master.uri;

import java.net.URI;
import java.util.concurrent.TimeUnit;

/**
 * A {@link MasterUriProvider} which will always return the same URI.
 *
 * @author Keith M. Hughes
 */
public class StaticMasterUriProvider implements MasterUriProvider {
  
  /**
   * The URI which will always be returned.
   */
  private final URI uri;

  public StaticMasterUriProvider(URI uri) {
    this.uri = uri;
  }

  @Override
  public URI getMasterUri() {
    return uri;
  }

  @Override
  public URI getMasterUri(long timeout, TimeUnit unit) {
    return uri;
  }
}
