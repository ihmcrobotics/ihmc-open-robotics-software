package org.ros.node;

public abstract class AbstractNodeMain implements NodeMain {
   public AbstractNodeMain() {
   }

   public void onStart(ConnectedNode connectedNode) {
   }

   public void onShutdown(Node node) {
   }

   public void onShutdownComplete(Node node) {
   }

   public void onError(Node node, Throwable throwable) {
   }
}