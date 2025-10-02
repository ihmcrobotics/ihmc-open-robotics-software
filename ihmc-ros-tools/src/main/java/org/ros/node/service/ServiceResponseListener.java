package org.ros.node.service;

import org.ros.exception.RemoteException;

public interface ServiceResponseListener<MessageType> {
   void onSuccess(MessageType var1);

   void onFailure(RemoteException var1);
}