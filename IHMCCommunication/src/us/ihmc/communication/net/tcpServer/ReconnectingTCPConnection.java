package us.ihmc.communication.net.tcpServer;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.communication.net.ConnectionStateListener;

public abstract class ReconnectingTCPConnection
{
   protected enum Status
   {
      CONNECTED,
      DISCONNECTED, 
      CLOSED
   }
   
   private final ReentrantLock writeLock = new ReentrantLock(true);
   private volatile Status status = Status.DISCONNECTED;
   protected final Object connectionStatusSync = new Object();
   protected final List<ConnectionStateListener> listeners = Collections.synchronizedList(new ArrayList<ConnectionStateListener>());
   protected final byte[] buffer;
   protected DataInputStream inputStream;
   protected OutputStream outputStream;
   private int offset = 0;

   public ReconnectingTCPConnection(int bufferSize)
   {
      buffer = new byte[bufferSize];
   }
   
   public int getBytesRead()
   {
      return offset;
   }

   public void read(int numberOfBytes) throws DisconnectedException
   {
      synchronized (connectionStatusSync)
      {
         if(status == Status.DISCONNECTED)
         {
            connect();
         }         
      }
      try
      {
         inputStream.readFully(buffer, offset, numberOfBytes);
         offset += numberOfBytes;
      }
      catch (IOException e)
      {
         System.err.println("Socket disconnected: " + e.getMessage());
         disconnect();
         throw new DisconnectedException();
      }
   }
   
   public void reset()
   {
      offset = 0;
   }
   
   public byte[] getBuffer()
   {
      return buffer;
   }

   public void write(byte[] value) throws DisconnectedException
   {
      if(status == Status.DISCONNECTED)
      {
         throw new DisconnectedException();
      }
      
      writeLock.lock();
      try
      {
         outputStream.write(value);
         outputStream.flush();
      }
      catch (IOException e)
      {
         System.err.println("Socket disconnected: " + e.getMessage());
         disconnect();
         throw new DisconnectedException();
      }
      finally
      {
         writeLock.unlock();
      }
   }
   
   protected Status getStatus()
   {
      synchronized (connectionStatusSync)
      {
         return status;
      }
   }
   
   protected void setStatus(Status status)
   {
      synchronized (connectionStatusSync)
      {
         this.status = status;
      }
   }
   
   public boolean isConnected()
   {
      return status == Status.CONNECTED;  // Not synchronized, otherwise potential deadlocks 
   }

   public void attachStateListener(ConnectionStateListener stateListener)
   {
      listeners.add(stateListener);
      if(isConnected())
      {
         stateListener.connected();
      }
   }
   
   protected void notifyConnectedListeners()
   {
      for(ConnectionStateListener listener : listeners)
      {
         listener.connected();
      }
   }
   
   protected void notifyDisconnectedListeners()
   {
      for(ConnectionStateListener listener : listeners)
      {
         listener.disconnected();
      }
   }
   
   public abstract void disconnect();
   protected abstract void connect();
   public abstract void close();
}
