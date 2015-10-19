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

/**
 * Fields found inside the header for node to node communication.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface ConnectionHeaderFields {

  public static final String CALLER_ID = "callerid";
  public static final String TOPIC = "topic";
  public static final String MD5_CHECKSUM = "md5sum";
  public static final String TYPE = "type";
  public static final String SERVICE = "service";
  public static final String TCP_NODELAY = "tcp_nodelay";
  public static final String LATCHING = "latching";
  public static final String PERSISTENT = "persistent";
  public static final String MESSAGE_DEFINITION = "message_definition";
  public static final String ERROR = "error";
  public static final String PROBE = "probe";
}
