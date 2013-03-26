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

import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.MessageBuffers;

import java.nio.charset.Charset;
import java.util.Collections;
import java.util.Map;
import java.util.Map.Entry;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ConnectionHeader {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(ConnectionHeader.class);

  private final Map<String, String> fields;

  /**
   * Decodes a header that came over the wire into a {@link Map} of fields and
   * values.
   * 
   * @param buffer
   *          the incoming {@link ChannelBuffer} containing the header
   * @return a {@link Map} of header fields and values
   */
  public static ConnectionHeader decode(ChannelBuffer buffer) {
    Map<String, String> fields = Maps.newHashMap();
    int position = 0;
    int readableBytes = buffer.readableBytes();
    while (position < readableBytes) {
      int fieldSize = buffer.readInt();
      position += 4;
      if (fieldSize == 0) {
        throw new IllegalStateException("Invalid 0 length handshake header field.");
      }
      if (position + fieldSize > readableBytes) {
        throw new IllegalStateException("Invalid line length handshake header field.");
      }
      String field = decodeAsciiString(buffer, fieldSize);
      position += field.length();
      Preconditions.checkState(field.indexOf("=") > 0,
          String.format("Invalid field in handshake header: \"%s\"", field));
      String[] keyAndValue = field.split("=");
      if (keyAndValue.length == 1) {
        fields.put(keyAndValue[0], "");
      } else {
        fields.put(keyAndValue[0], keyAndValue[1]);
      }
    }
    if (DEBUG) {
      log.info("Decoded header: " + fields);
    }
    ConnectionHeader connectionHeader = new ConnectionHeader();
    connectionHeader.mergeFields(fields);
    return connectionHeader;
  }

  private static String decodeAsciiString(ChannelBuffer buffer, int length) {
    return buffer.readBytes(length).toString(Charset.forName("US-ASCII"));
  }

  public ConnectionHeader() {
    this.fields = Maps.newConcurrentMap();
  }

  /**
   * Encodes this {@link ConnectionHeader} for transmission over the wire.
   * 
   * @return a {@link ChannelBuffer} containing the encoded header for wire
   *         transmission
   */
  public ChannelBuffer encode() {
    ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
    for (Entry<String, String> entry : fields.entrySet()) {
      String field = entry.getKey() + "=" + entry.getValue();
      buffer.writeInt(field.length());
      buffer.writeBytes(field.getBytes(Charset.forName("US-ASCII")));
    }
    return buffer;
  }

  public void merge(ConnectionHeader other) {
    Map<String, String> otherFields = other.getFields();
    mergeFields(otherFields);
  }

  public void mergeFields(Map<String, String> other) {
    for (Entry<String, String> field : other.entrySet()) {
      String name = field.getKey();
      String value = field.getValue();
      addField(name, value);
    }
  }

  public void addField(String name, String value) {
    if (!fields.containsKey(name) || fields.get(name).equals(value)) {
      fields.put(name, value);
    } else {
      throw new RosRuntimeException(String.format("Unable to merge field %s: %s != %s", name,
          value, fields.get(name)));
    }
  }

  public Map<String, String> getFields() {
    return Collections.unmodifiableMap(fields);
  }

  public boolean hasField(String name) {
    return fields.containsKey(name);
  }

  public String getField(String name) {
    return fields.get(name);
  }

  @Override
  public String toString() {
    return String.format("ConnectionHeader <%s>", fields.toString());
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((fields == null) ? 0 : fields.hashCode());
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
    ConnectionHeader other = (ConnectionHeader) obj;
    if (fields == null) {
      if (other.fields != null)
        return false;
    } else if (!fields.equals(other.fields))
      return false;
    return true;
  }
}
