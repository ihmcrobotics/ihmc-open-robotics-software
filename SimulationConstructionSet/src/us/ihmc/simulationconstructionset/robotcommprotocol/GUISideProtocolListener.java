package us.ihmc.simulationconstructionset.robotcommprotocol;

import java.io.DataInputStream;
import java.io.EOFException;
import java.io.IOException;
import java.net.SocketTimeoutException;
import java.util.ArrayList;

import us.ihmc.simulationconstructionset.NewDataListener;

public class GUISideProtocolListener implements UserCommandNetworkReader
{
   private final boolean DEBUG = false;
   
   private final DataInputStream dataInputStream;
   private final ArrayList<NewDataListener> newDataListeners;
   private final GUISideAbstractCommandListener guiSideCommandListener;

   private static final byte IN_SYNC_BYTE = (byte) 33;
   private static final byte OUT_SYNC_BYTE = (byte) 79;

   public GUISideProtocolListener(DataInputStream dataIn, GUISideAbstractCommandListener commandListener)
   {
      this(dataIn, commandListener, null);
   }

   public GUISideProtocolListener(DataInputStream dataIn, GUISideAbstractCommandListener commandListener, ArrayList<NewDataListener> newDataListeners)
   {
      if (DEBUG) System.out.println("Creating RobotProtocolListener. This one Sends SYNC Bytes!!");
      this.newDataListeners = newDataListeners;
      this.dataInputStream = dataIn;
      this.guiSideCommandListener = commandListener;
   }

   public void processInput() throws IOException
   {
      if (DEBUG)
      {
         System.out.println("GUISideProtocolListener in processInput()");
      }
      
      byte syncByte;

      try
      {
         syncByte = dataInputStream.readByte();
      }
      catch (SocketTimeoutException exception)
      {
         System.err.println("Socket Timed Out...");    // +++JEP 11/26

         // If timed out while waiting for input, just return for now...
         return;
      }
      catch (EOFException exception)
      {
         if (DEBUG)
         {
            System.out.println("GUISideProtocolListener: Received EOFException!");
         }
         
         return;
      }

      if (DEBUG)
      {
         System.out.println("GUISideProtocolListener: Received syncByte: " + syncByte);
      }
      
      try
      {
         if (syncByte != IN_SYNC_BYTE)
         {
            System.out.println("Bad IN SYNC: " + syncByte + " should be " + IN_SYNC_BYTE);
            System.out.flush();

//          System.exit( -1);

            byte lastSyncByte = 0;
            while ((lastSyncByte != OUT_SYNC_BYTE) && (syncByte != IN_SYNC_BYTE))
            {
               lastSyncByte = syncByte;
               syncByte = dataInputStream.readByte();
               System.out.println(syncByte);
               System.out.flush();
            }
         }

         int commandInt = dataInputStream.readInt();
         RobotProtocolCommand command = RobotProtocolCommand.values()[commandInt];

         if (DEBUG)
         {
            System.out.println("GUISideProtocolListener: Received Command: " + command);
         }

         
         switch (command)
         {
            case HELLO :
            {
               String name = dataInputStream.readUTF();
               String info = dataInputStream.readUTF();

               syncByte = dataInputStream.readByte();
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doHello(name, info);
            }

            break;

            case ALL_REGISTRIES_AND_VARIABLES :
            {
               int nRegistries = dataInputStream.readInt();
               String[] registryNames = new String[nRegistries];
               String[][] variableNames = new String[nRegistries][];
               float[][] initialValues = new float[nRegistries][];
               
               for (int i = 0; i < nRegistries; i++)
               {
                  registryNames[i] = dataInputStream.readUTF();
                  
                  int numberOfVariablesInRegistry = dataInputStream.readInt();
                  variableNames[i] = new String[numberOfVariablesInRegistry];
                  initialValues[i] = new float[numberOfVariablesInRegistry];

                  for (int j=0; j<numberOfVariablesInRegistry; j++)
                  {
                     String variableName = dataInputStream.readUTF();
                     variableNames[i][j] = variableName;
                     initialValues[i][j] = dataInputStream.readFloat();
                  }
               }

               syncByte = dataInputStream.readByte();
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doAllRegistriesAndVariables(registryNames, variableNames, initialValues);
            }

            break;

            case REGISTRY_SETTINGS_PROCESSED :
            {
               int registrySettingsIdentifier = dataInputStream.readInt();
               
               int size = dataInputStream.readInt();
               int[] indices = new int[size];
               boolean[] isSent = new boolean[size];
               boolean[] isDisallowSendingSet = new boolean[size];
               boolean[] isLogged = new boolean[size];
               
               for (int i = 0; i < size; i++)
               {
                  indices[i] = dataInputStream.readInt();
                  isSent[i] = dataInputStream.readBoolean();
                  isDisallowSendingSet[i] = dataInputStream.readBoolean();
                  isLogged[i] = dataInputStream.readBoolean();
               }
                      
               syncByte = dataInputStream.readByte();
               
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doRegistrySettingsProcessed(indices, isSent, isDisallowSendingSet, isLogged, registrySettingsIdentifier);
            }

            break;

            case SET :
            {
               int index = dataInputStream.readInt();
               float val = dataInputStream.readFloat();

               syncByte = dataInputStream.readByte();
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doSet(index, val);
            }

            break;

            case PERIOD :
            {
               int periodmsec = dataInputStream.readInt();

               syncByte = dataInputStream.readByte();
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doPeriod(periodmsec);
            }

            break;

            case DISCONNECT :
            {
               syncByte = dataInputStream.readByte();
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doDisconnect();
            }

            break;

            case USR_CMD :
            {
               String cmd = dataInputStream.readUTF();

               syncByte = dataInputStream.readByte();
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doUserCommand(cmd);
            }

            break;

            case DATA :
            {
               int nvars = dataInputStream.readInt();
               float[] data = new float[nvars];
               for (int i = 0; i < nvars; i++)
               {
                  data[i] = dataInputStream.readFloat();
               }

               syncByte = dataInputStream.readByte();
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doData(data);

               if (newDataListeners != null)
               {
                  for (NewDataListener listener : newDataListeners)
                  {
                     listener.newDataHasBeenReceived();
                  }
               }
            }

            break;

            case TEXT_MESSAGE :
            {
               String message = dataInputStream.readUTF();

               syncByte = dataInputStream.readByte();
               if (syncByte != OUT_SYNC_BYTE)
                  throw new RuntimeException("Bad Out Sync: " + syncByte + ". Should be " + OUT_SYNC_BYTE);
               else
                  guiSideCommandListener.doTextMessage(message);
            }

            break;

            default :
            {
               System.err.println("Invalid Command: " + command);
            }
         }
      }
      catch (SocketTimeoutException e)
      {
         System.err.println("Caught SocketTimeoutException. Disconnecting.");
         guiSideCommandListener.doDisconnect();
      }
   }

   

   @Override
   public int readInt() throws IOException
   {
      return dataInputStream.readInt();
   }

   @Override
   public float readFloat() throws IOException
   {
      return dataInputStream.readFloat();
   }

   @Override
   public double readDouble() throws IOException
   {
      return dataInputStream.readDouble();
   }
}
