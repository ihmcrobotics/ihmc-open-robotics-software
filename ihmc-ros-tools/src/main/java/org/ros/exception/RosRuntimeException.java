package org.ros.exception;

public class RosRuntimeException extends RuntimeException {
   public RosRuntimeException(Throwable throwable) {
      super(throwable);
   }

   public RosRuntimeException(String message, Throwable throwable) {
      super(message, throwable);
   }

   public RosRuntimeException(String message) {
      super(message);
   }
}