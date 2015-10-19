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

package org.ros.message;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface MessageSerializationFactory {

  /**
   * @param messageType
   *          the type of message that the new {@link MessageSerializer} should
   *          serialize
   * @return a new {@link MessageSerializer} for the provided message type
   */
  <T> MessageSerializer<T> newMessageSerializer(String messageType);

  /**
   * @param messageType
   *          the type of message that the new {@link MessageDeserializer}
   *          should deserialize
   * @return a new {@link MessageDeserializer} for the provided message type
   */
  <T> MessageDeserializer<T> newMessageDeserializer(String messageType);

  /**
   * @param serviceType
   *          the type of service that the new {@link MessageSerializer} should
   *          serialize requests for
   * @return a new {@link MessageSerializer} for requests to the provided
   *         service type
   */
  <T> MessageSerializer<T> newServiceRequestSerializer(String serviceType);

  /**
   * @param serviceType
   *          the type of service that the new {@link MessageDeserializer}
   *          should deserialize requests for
   * @return a new {@link MessageDeserializer} for requests to the provided
   *         service type
   */
  <T> MessageDeserializer<T> newServiceRequestDeserializer(String serviceType);

  /**
   * @param serviceType
   *          the type of service that the new {@link MessageSerializer} should
   *          serialize responses for
   * @return a new {@link MessageSerializer} for responses from the provided
   *         service type
   */
  <T> MessageSerializer<T> newServiceResponseSerializer(String serviceType);

  /**
   * @param serviceType
   *          the type of service that the new {@link MessageDeserializer}
   *          should deserialize responses for
   * @return a new {@link MessageDeserializer} for responses from the provided
   *         service type
   */
  <T> MessageDeserializer<T> newServiceResponseDeserializer(String serviceType);
}
