package us.ihmc.simulationConstructionSetTools.socketCommunication;

import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.NewDataListener;

public class GUISideProtocolTalker
{
   private final DataOutputStream dataOutputStream;

   private static final byte IN_SYNC_BYTE = (byte) 33;
   private static final byte OUT_SYNC_BYTE = (byte) 79;

   private final ArrayList<NewDataListener> newDataListeners;

   public GUISideProtocolTalker(DataOutputStream dataOutputStream, ArrayList<NewDataListener> newDataListeners)
   {
      this.dataOutputStream = dataOutputStream;
      this.newDataListeners = newDataListeners;
   }

   public void sendHello(String name, String info)
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.HELLO.ordinal());
         dataOutputStream.writeUTF(name);
         dataOutputStream.writeUTF(info);
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void sendRequestAllRegistriesAndVariables()
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.REQ_ALL_REGISTRIES_AND_VARIABLES.ordinal());
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void sendRegistrySettings(HashMap<YoVariableRegistry, Integer> registryIndexMap, int registrySettingsIdentifier)
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.REGISTRY_SETTINGS.ordinal());
         dataOutputStream.writeInt(registrySettingsIdentifier);
         dataOutputStream.writeInt(registryIndexMap.size());

         for (YoVariableRegistry registry : registryIndexMap.keySet())
         {
            dataOutputStream.writeInt(registryIndexMap.get(registry));
            dataOutputStream.writeBoolean(registry.isSent());
//            dataOutputStream.writeBoolean(registry.isDisallowSendingSet());
            dataOutputStream.writeBoolean(registry.isLogged());
         }

         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void sendUpdateVariables()
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.UPDATE_VARS.ordinal());
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public boolean sendCheckConnected()
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.CHECK_CONNECTED.ordinal());
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (Exception e)
      {
         return false;
      }

      return true;
   }

   public void sendSet(int index, float value)
   {
//    System.out.println("GUISideProtocolTalker: Sending set");
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.SET.ordinal());
         dataOutputStream.writeInt(index);
         dataOutputStream.writeFloat(value);
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();

         if (newDataListeners != null)
         {
            for (NewDataListener listener : newDataListeners)
            {
               listener.newDataHasBeenSent();
            }
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void sendPeriod(int periodmsec)
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.PERIOD.ordinal());
         dataOutputStream.writeInt(periodmsec);
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void sendDisconnect()
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.DISCONNECT.ordinal());
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void sendUserCommand(String cmd)
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.USR_CMD.ordinal());
         dataOutputStream.writeUTF(cmd);
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void sendData(int listid, ArrayList<YoVariable<?>> vars)
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.DATA.ordinal());
         dataOutputStream.writeInt(listid);
         dataOutputStream.writeInt(vars.size());

         for (int i = 0; i < vars.size(); i++)
         {
            YoVariable<?> v = vars.get(i);
            dataOutputStream.writeFloat((float) v.getValueAsDouble());
         }

         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();

      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void sendTextMessage(String message)
   {
      try
      {
         dataOutputStream.writeByte(IN_SYNC_BYTE);
         dataOutputStream.writeInt(RobotProtocolCommand.TEXT_MESSAGE.ordinal());
         dataOutputStream.writeUTF(message);
         dataOutputStream.writeByte(OUT_SYNC_BYTE);
         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

}
