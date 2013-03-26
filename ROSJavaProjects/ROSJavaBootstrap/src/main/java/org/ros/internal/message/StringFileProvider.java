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

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

import org.apache.commons.io.DirectoryWalker;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.apache.commons.io.filefilter.IOFileFilter;
import org.ros.exception.RosRuntimeException;

import java.io.File;
import java.io.FileFilter;
import java.io.IOException;
import java.util.Collection;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class StringFileProvider {

  private final Collection<File> directories;
  private final Map<File, String> strings;
  private final StringFileDirectoryWalker stringFileDirectoryWalker;

  private final class StringFileDirectoryWalker extends DirectoryWalker {
    
    private final Set<File> directories;

    private StringFileDirectoryWalker(FileFilter filter, int depthLimit) {
      super(filter, depthLimit);
      directories = Sets.newHashSet();
    }
    
    // TODO(damonkohler): Update Apache Commons IO to the latest version.
    @SuppressWarnings("rawtypes")
    @Override
    protected boolean handleDirectory(File directory, int depth, Collection results)
        throws IOException {
      File canonicalDirectory = directory.getCanonicalFile();
      if (directories.contains(canonicalDirectory)) {
        return false;
      }
      directories.add(canonicalDirectory);
      return true;
    }

    @SuppressWarnings("rawtypes")
    @Override
    protected void handleFile(File file, int depth, Collection results) {
      String content;
      try {
        content = FileUtils.readFileToString(file, "US-ASCII");
      } catch (IOException e) {
        throw new RosRuntimeException(e);
      }
      strings.put(file, content);
    }

    public void update(File directory) {
      try {
        walk(directory, null);
      } catch (IOException e) {
        throw new RosRuntimeException(e);
      }
    }
  }

  public StringFileProvider(IOFileFilter ioFileFilter) {
    directories = Lists.newArrayList();
    strings = Maps.newConcurrentMap();
    IOFileFilter directoryFilter = FileFilterUtils.directoryFileFilter();
    FileFilter fileFilter = FileFilterUtils.orFileFilter(directoryFilter, ioFileFilter);
    stringFileDirectoryWalker = new StringFileDirectoryWalker(fileFilter, -1);
  }

  public void update() {
    for (File directory : directories) {
      stringFileDirectoryWalker.update(directory);
    }
  }

  /**
   * Adds a new directory to be scanned for topic definition files.
   * 
   * @param directory
   *          the directory to add
   */
  public void addDirectory(File directory) {
    Preconditions.checkArgument(directory.isDirectory());
    directories.add(directory);
  }

  public Map<File, String> getStrings() {
    return ImmutableMap.copyOf(strings);
  }

  public String get(File file) {
    if (!has(file)) {
      throw new NoSuchElementException("File does not exist: " + file);
    }
    return strings.get(file);
  }

  public boolean has(File file) {
    return strings.containsKey(file);
  }
}
