package us.ihmc.simulationconstructionset.robotcommprotocol;

import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.NewDataListener;
import us.ihmc.simulationconstructionset.gui.YoEntryBox;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanel;

public class RobotSocketConnection implements RobotConnection, VariableChangedListener, DoDisconnectListener, RegistrySettingsChangedListener
{
   private Socket ethernetConnection;

   private GUISideProtocolListenerThread robotProtocolListenerThread;
   private GUISideProtocolListener protocolListener;
   private GUISideProtocolTalker guiSideProtocolTalker;

   private ArrayList<NewDataListener> newDataListeners;
   private int registrySettingsIdentifier = 0;
   private final String host;
   public static int PORT = 24608;

   private GUISideCommandListener commandListener;

   public RobotSocketConnection(String host, GUISideCommandListener commandListener, YoVariableRegistry rootRegistry)
   {
      this.host = host;
      this.commandListener = commandListener;
   }

   public RobotSocketConnection(String host, GUISideCommandListener commandListener, YoVariableRegistry rootRegistry,
                                ArrayList<NewDataListener> newDataListeners)
   {
      this(host, commandListener, rootRegistry);
      if (newDataListeners != null)
         this.addNewDataListeners(newDataListeners);
   }

   public RobotSocketConnection(String host, GUISideCommandListener commandListener, YoVariableRegistry rootRegistry, NewDataListener newDataListener)
   {
      this(host, commandListener, rootRegistry);
      if (newDataListener != null)
         this.addNewDataListener(newDataListener);
   }

   private void addNewDataListener(NewDataListener listener)
   {
      if (newDataListeners == null)
         newDataListeners = new ArrayList<NewDataListener>();
      newDataListeners.add(listener);
   }

   private void addNewDataListeners(ArrayList<NewDataListener> listeners)
   {
      if (newDataListeners == null)
         newDataListeners = new ArrayList<NewDataListener>();
      newDataListeners.addAll(listeners);
   }

   private synchronized boolean setupEthernetConnection()
   {
      boolean ret = false;

      try
      {
         // System.out.println("Trying to Connect to " + HOST + "PORT: " + PORT);
         ethernetConnection = new Socket(host, PORT);

         // System.out.println("Trying to Create a listener");
         // protocolListener = new RobotProtocolListener(ethernetConnection, commandListener);
         ret = true;
      }
      catch (UnknownHostException e)
      {
         System.err.println("Could not connect to " + host + ". " + e);
      }
      catch (IOException e)
      {
         System.err.println("Could not connect to " + host + ". " + e);
      }

      return ret;
   }

   @Override
   public boolean isConnected()
   {
      return commandListener.isConnected();
   }

   @Override
   public void pause()
   {
      if (!commandListener.isConnected())
         return;
      robotProtocolListenerThread.pause();
   }

   @Override
   public synchronized void attemptConnectionToHost()
   {
      if (!commandListener.isConnected())
      {
         System.out.println("Trying to Connect to " + host);

         if (setupEthernetConnection())
         {
            robotProtocolListenerThread = new GUISideProtocolListenerThread(ethernetConnection, commandListener, newDataListeners);

            // protocolListener = new RobotProtocolListener(ethernetConnection, commandListener, sendIndexMap);
            protocolListener = robotProtocolListenerThread.getGUISideProtocolListener();

            guiSideProtocolTalker = new GUISideProtocolTalker(robotProtocolListenerThread.getDataOutputStream(), newDataListeners);
         }
         else
            return;

         // System.out.println("Trying to Create a Serial listener");
         // if (setupSerialConnection()) protocolListener = new RobotProtocolListener(serialConnection, commandListener);
         // else return;

         System.out.println("Sending a few messages");

         // Send a few messages:

         System.out.println("Sending Hello");
         guiSideProtocolTalker.sendHello("Dumb PC Emulator", "Hello there!!");

         while (!commandListener.isConnected())
         {
            try
            {
               Thread.sleep(10);
            }
            catch (InterruptedException e)
            {
            }
         }

         System.out.println("Sending Period");
         guiSideProtocolTalker.sendPeriod(500);    // 1500);

         System.out.println("Sending Request All Registries And Variables");
         guiSideProtocolTalker.sendRequestAllRegistriesAndVariables();

         while (!commandListener.isDoneReceivingAllRegistriesAndVariables())
         {
            try
            {
               Thread.sleep(100);
            }
            catch (InterruptedException e)
            {
            }
         }

         System.out.println("Done receiving all registries and variables!");

         sendRegistrySettings();

         // Attach protocolListener as the change listener to anything the user can change a variable with...
         YoEntryBox.attachVariableChangedListener(this);
         YoVariablePanel.attachVariableChangedListener(this);

//       this.setText("Disconnect");

      }
      else
      {
         // connected = false;
         // this.setText("Connect");
         guiSideProtocolTalker.sendDisconnect();

         // Don't disconnect until receive reply...

         /*
          * protocolListener.disconnect();
          *      if (ethernetConnection != null)
          *      {
          * try{ethernetConnection.close();}
          * catch(IOException e){}
          *      }
          */
      }

      return;
   }

   private void sendRegistrySettings()
   {
      registrySettingsIdentifier++;

      if (guiSideProtocolTalker != null)
      {
         commandListener.expectNewRegistrySettings(registrySettingsIdentifier);
//         commandListener.updateSendVars();
//         commandListener.updateLogVarsCount();
//
//         System.out.println("Sending registry settings. Send vars: " + commandListener.getNumberOfSendVars() + ". Log vars: "
//                            + commandListener.getNumberOfLogVars());

         guiSideProtocolTalker.sendRegistrySettings(commandListener.getRegistryIndexMap(), registrySettingsIdentifier);
      }
   }

   public boolean getRegistrySettingsProcessed()
   {
      return commandListener.getRegistrySettingsProcessed();
   }

   private void sendRegistrySettings(ArrayList<YoVariableRegistry> registriesToSendSettingsOf)
   {
      registrySettingsIdentifier++;

      if (guiSideProtocolTalker != null)
      {
//         commandListener.updateSendVars(); 
//         commandListener.updateLogVarsCount();
         
         LinkedHashMap<YoVariableRegistry, Integer> registryIndexMapSubset = new LinkedHashMap<YoVariableRegistry, Integer>();
         for (YoVariableRegistry registry : registriesToSendSettingsOf)
         {
            try
            {
               int index = commandListener.getIndex(registry);

               registryIndexMapSubset.put(registry, index);
            }
            catch (Exception exception)
            {
               System.err.println("Trouble in sendRegistrySettings with registry " + registry.getName() + ". Most likely that you are trying to send settings for a registry that is on the GUI side, but not on the robot side.");
            }
         }

//         System.out.println("Sending registry settings. Send vars: " + commandListener.getNumberOfSendVars() + ". Log vars: "
//               + commandListener.getNumberOfLogVars());

         commandListener.expectNewRegistrySettings(registrySettingsIdentifier);
         guiSideProtocolTalker.sendRegistrySettings(registryIndexMapSubset, registrySettingsIdentifier);
      }
   }

   @Override
   public void doDisconnect()
   {
      disconnect();

      if (commandListener.isConnected())
      {
         if (robotProtocolListenerThread != null)
         {
            robotProtocolListenerThread.disconnect();
            robotProtocolListenerThread = null;
         }

         if (ethernetConnection != null)
         {
            try
            {
               ethernetConnection.close();
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }

            ethernetConnection = null;
         }
      }
   }

   @Override
   public void disconnect()
   {
      if (commandListener.isConnected())
      {
         guiSideProtocolTalker.sendDisconnect();
      }

      while (commandListener.isConnected())
      {
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException e)
         {
         }
      }

   }

   @Override
   public void setRecord(boolean record)
   {
      commandListener.setRecord(record);
   }

   @Override
   public void variableChanged(YoVariable<?> variable)
   {
//    System.out.println("commandListener.isConnected(): " + commandListener.isConnected());
//    System.out.println("protocolListener: " + protocolListener);
      if (GUISideCommandListener.isRecording())
      {
         if (commandListener.isConnected() && (protocolListener != null))
            guiSideProtocolTalker.sendSet(commandListener.getIndex(variable), (float) variable.getValueAsDouble());
      }
   }

   @Override
   public void registrySettingsChanged(ArrayList<YoVariableRegistry> changedRegistries)
   {
//      sendRegistrySettings();
      if (commandListener.isConnected())
      {
         sendRegistrySettings(changedRegistries);
      }
   }

   @Override
   public void registrySettingsChanged()
   {
      sendRegistrySettings();
   }
}
