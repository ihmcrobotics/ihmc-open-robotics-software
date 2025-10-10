package org.ros.exception;

//import org.ros.internal.node.response.StatusCode;

public class RemoteException extends RosRuntimeException {
//   private final StatusCode statusCode;
//
//   public RemoteException(StatusCode statusCode, String message) {
//      super(message);
//      this.statusCode = statusCode;
//   }

   public RemoteException(String message)
   {
      super(message);
   }

   //   public StatusCode getStatusCode() {
//      return this.statusCode;
//   }
}