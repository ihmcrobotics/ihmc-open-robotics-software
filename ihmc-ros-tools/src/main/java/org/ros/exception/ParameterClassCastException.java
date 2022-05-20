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

package org.ros.exception;

/**
 * Thrown when a requested parameter does not match the requested parameter
 * type.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ParameterClassCastException extends RosRuntimeException {

  public ParameterClassCastException(String message) {
    super(message);
  }

  public ParameterClassCastException(String message, Throwable throwable) {
    super(message, throwable);
  }

  public ParameterClassCastException(Throwable throwable) {
    super(throwable);
  }
}
