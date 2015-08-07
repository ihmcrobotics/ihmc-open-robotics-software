package us.ihmc.robotiq.communication;

import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;

import net.wimpi.modbus.ModbusException;
import net.wimpi.modbus.ModbusIOException;
import net.wimpi.modbus.ModbusSlaveException;
import net.wimpi.modbus.io.ModbusTCPTransaction;
import net.wimpi.modbus.msg.ReadInputRegistersRequest;
import net.wimpi.modbus.msg.ReadInputRegistersResponse;
import net.wimpi.modbus.msg.WriteMultipleRegistersRequest;
import net.wimpi.modbus.net.TCPMasterConnection;
import net.wimpi.modbus.procimg.InputRegister;
import net.wimpi.modbus.procimg.Register;
import us.ihmc.tools.thread.ThreadTools;

public class JamodTCPMaster
{
   private final boolean DEBUG = false;
   
   private InetAddress slaveAddress;
   private TCPMasterConnection connection;
   private ModbusTCPTransaction transaction;
   private WriteMultipleRegistersRequest writeMultipleRegistersRequest;
   private ReadInputRegistersRequest readInputRegistersRequest;
   
   private boolean autoreconnect = false;

   public JamodTCPMaster(String address)
   {
      try
      {
         slaveAddress = InetAddress.getByName(address);
         connection = new TCPMasterConnection(slaveAddress);
         connection.setTimeout(3000);
         writeMultipleRegistersRequest = new WriteMultipleRegistersRequest();
         readInputRegistersRequest = new ReadInputRegistersRequest();
         readInputRegistersRequest.setUnitID(2);
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }
      
      connect();
   }
   
   public JamodTCPMaster(String address, int port)
   {
      this(address);
      connection.setPort(port);
   }
   
  public void connect()
   {
      if(!connection.isConnected())
      {
         try
         {
            connection.connect();
            transaction = new ModbusTCPTransaction(connection);
            transaction.setRetries(0);
            System.out.println("Successfully connected at " + connection.getAddress().getHostAddress());
         }
         catch (Exception e)
         {
            System.out.println(getClass().getSimpleName() + ": Unable to connect to " + slaveAddress.getHostAddress());
         }
      }
   }
   
   public void disconnect()
   {
      if(connection.isConnected())
      {
         connection.close();
         transaction = null;
      }
   }
   
   public boolean isConnected()
   {
      return connection.isConnected();
   }
   
   public void reconnect()
   {
      disconnect();
      ThreadTools.sleep(50);
      connect();
   }
   
   public void setAutoReconnect(boolean autoreconnect)
   {
      if(this.autoreconnect != autoreconnect)
      {
         this.autoreconnect = autoreconnect;
         if(autoreconnect)
            new Thread(new ConnectionListener()).start();
      }
   }
   
   public synchronized InputRegister[] readInputRegisters(int offset, int numberOfRegistersToRead) throws ModbusIOException, ModbusSlaveException, ModbusException, SocketTimeoutException
   {
      readInputRegistersRequest.setReference(offset);
      readInputRegistersRequest.setWordCount(numberOfRegistersToRead);
      transaction.setRequest(readInputRegistersRequest);
      transaction.execute();
      
      if(DEBUG)
      {
         System.out.println("Request:");
         System.out.println(readInputRegistersRequest.getHexMessage());
         System.out.println("Response:");
         System.out.println(((ReadInputRegistersResponse)transaction.getResponse()).getHexMessage());
      }
      
      return ((ReadInputRegistersResponse)transaction.getResponse()).getRegisters();
   }
   
   public synchronized void writeMultipleRegisters(int offset, Register[] registers) throws ModbusIOException, ModbusSlaveException, ModbusException, SocketTimeoutException
   {
      writeMultipleRegistersRequest.setReference(offset);
      writeMultipleRegistersRequest.setRegisters(registers);
      transaction.setRequest(writeMultipleRegistersRequest);
      transaction.execute();
   }
   
   class ConnectionListener implements Runnable
   {
      public void run()
      {
         while(autoreconnect)
         {
            if(isConnected())
            {
               try
               {
                  readInputRegisters(0, 1);
               }
               catch (ModbusException | SocketTimeoutException e)
               {
                  System.out.println(getClass().getSimpleName() + ": " + slaveAddress.getHostAddress() + " disconnected. Attempting to reconnect...");
                  reconnect();
               }
               
               ThreadTools.sleep(200);
            }
         }
      }
   }
   
}
