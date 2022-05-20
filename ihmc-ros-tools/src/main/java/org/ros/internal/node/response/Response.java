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

package org.ros.internal.node.response;

import com.google.common.collect.Lists;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;

import java.util.List;

/**
 * The response from an XML-RPC call.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <T>
 */
public class Response<T> {

  private final StatusCode statusCode;
  private final String statusMessage;
  private final T result;

  public static <T> Response<T> newError(String message, T value) {
    return new Response<T>(StatusCode.ERROR, message, value);
  }

  public static <T> Response<T> newFailure(String message, T value) {
    return new Response<T>(StatusCode.FAILURE, message, value);
  }

  public static <T> Response<T> newSuccess(String message, T value) {
    return new Response<T>(StatusCode.SUCCESS, message, value);
  }

  /**
   * Creates a {@link Response} from the {@link List} of {@link Object}s
   * returned from an XML-RPC call. Throws {@link RemoteException} if the
   * {@link StatusCode} is StatusCode.FAILURE.
   * 
   * @param <T>
   * @param response
   *          the {@link List} of {@link Object}s returned from the XML-RPC call
   * @param resultFactory
   *          a {@link ResultFactory} that creates a result from the third
   *          {@link Object} in the {@link Response}
   * @return a {@link Response} using the specified {@link ResultFactory} to
   *         generate the result
   * @throws RemoteException
   *           if the {@link Response}'s {@link StatusCode} indicates
   *           StatusCode.FAILURE.
   */
  public static <T> Response<T> fromListCheckedFailure(List<Object> response,
      ResultFactory<T> resultFactory) throws RemoteException {
    StatusCode statusCode;
    String message;
    try {
      statusCode = StatusCode.fromInt((Integer) response.get(0));
      message = (String) response.get(1);
      if (statusCode == StatusCode.FAILURE) {
        throw new RemoteException(statusCode, message);
      }

    } catch (ClassCastException e) {
      throw new RosRuntimeException(
          "Remote side did not return correct type (status code/message).", e);
    }
    try {
      return new Response<T>(statusCode, message, resultFactory.newFromValue(response.get(2)));
    } catch (ClassCastException e) {
      throw new RosRuntimeException("Remote side did not return correct value type.", e);
    }
  }

  /**
   * Creates a {@link Response} from the {@link List} of {@link Object}s
   * returned from an XML-RPC call. Throws {@link RemoteException} if the
   * {@link StatusCode} is not a success.
   * 
   * @param <T>
   * @param response
   *          the {@link List} of {@link Object}s returned from the XML-RPC call
   * @param resultFactory
   *          a {@link ResultFactory} that creates a result from the third
   *          {@link Object} in the {@link Response}
   * @return a {@link Response} using the specified {@link ResultFactory} to
   *         generate the result
   * @throws RemoteException
   *           if the {@link Response}'s {@link StatusCode} does not indicate
   *           success
   */
  public static <T> Response<T> fromListChecked(List<Object> response,
      ResultFactory<T> resultFactory) throws RemoteException {
    StatusCode statusCode;
    String message;
    try {
      statusCode = StatusCode.fromInt((Integer) response.get(0));
      message = (String) response.get(1);
      if (statusCode != StatusCode.SUCCESS) {
        throw new RemoteException(statusCode, message);
      }
    } catch (ClassCastException e) {
      throw new RosRuntimeException(
          "Remote side did not return correct type (status code/message).", e);
    }
    try {
      return new Response<T>(statusCode, message, resultFactory.newFromValue(response.get(2)));
    } catch (ClassCastException e) {
      throw new RosRuntimeException("Remote side did not return correct value type.", e);
    }
  }

  public Response(int statusCode, String statusMessage, T value) {
    this(StatusCode.fromInt(statusCode), statusMessage, value);
  }

  public Response(StatusCode statusCode, String statusMessage, T value) {
    this.statusCode = statusCode;
    this.statusMessage = statusMessage;
    this.result = value;
  }

  public List<Object> toList() {
    return Lists.newArrayList(statusCode.toInt(), statusMessage, result == null ? "null" : result);
  }

  public StatusCode getStatusCode() {
    return statusCode;
  }

  public String getStatusMessage() {
    return statusMessage;
  }

  public T getResult() {
    return result;
  }

  @Override
  public String toString() {
    return "Response<" + statusCode + ", " + statusMessage + ", " + result + ">";
  }

  public boolean isSuccess() {
    return statusCode == StatusCode.SUCCESS;
  }
}
