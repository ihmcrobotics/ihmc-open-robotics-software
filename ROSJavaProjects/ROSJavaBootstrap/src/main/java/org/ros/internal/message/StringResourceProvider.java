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

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Maps;

import org.ros.exception.RosRuntimeException;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.Charset;
import java.util.Map;
import java.util.NoSuchElementException;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class StringResourceProvider {

  private final Map<String, String> cache;

  public StringResourceProvider() {
    cache = Maps.newConcurrentMap();
  }

  public String get(String resourceName) {
    if (!has(resourceName)) {
      throw new NoSuchElementException("Resource does not exist: " + resourceName);
    }
    if (!cache.containsKey(resourceName)) {
      InputStream in = getClass().getResourceAsStream(resourceName);
      StringBuilder out = new StringBuilder();
      Charset charset = Charset.forName("US-ASCII");
      byte[] buffer = new byte[8192];
      try {
        for (int bytesRead; (bytesRead = in.read(buffer)) != -1;) {
          out.append(new String(buffer, 0, bytesRead, charset));
        }
      } catch (IOException e) {
        throw new RosRuntimeException("Failed to read resource: " + resourceName, e);
      }
      cache.put(resourceName, out.toString());
    }
    return cache.get(resourceName);
  }

  public boolean has(String resourceName) {
    return cache.containsKey(resourceName) || getClass().getResource(resourceName) != null;
  }

  public Map<String, String> getCachedStrings() {
    return ImmutableMap.copyOf(cache);
  }

  public void addStringToCache(String resourceName, String resourceContent) {
    cache.put(resourceName, resourceContent);
  }
}
