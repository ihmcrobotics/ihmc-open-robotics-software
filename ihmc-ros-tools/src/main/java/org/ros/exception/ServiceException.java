package org.ros.exception;

public class ServiceException extends Exception {
   public ServiceException(Throwable throwable) {
      super(throwable);
   }

   public ServiceException(String message, Throwable throwable) {
      super(message, throwable);
   }

   public ServiceException(String message) {
      super(message);
   }
}