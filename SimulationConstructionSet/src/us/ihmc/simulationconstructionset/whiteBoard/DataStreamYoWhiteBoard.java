package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableType;


public abstract class DataStreamYoWhiteBoard extends YoWhiteBoard implements Runnable
{
   private static final boolean VERBOSE = false;

   protected static final int SYNC_IN = 107;
   protected static final int SYNC_OUT = 393;

   private DataInputStream dataInputStream = null;
   private DataOutputStream dataOutputStream = null;

   private final boolean writeOutConnect, readInConnect;
   private final boolean createYoVariablesOnConnect;
   private final YoVariableRegistry rootRegistryToAddVariablesTo;

   protected abstract void allowThrowOutStalePacketsIfYouWish();

   public DataStreamYoWhiteBoard(String name, boolean writeOutConnect, boolean readInConnect)
   {
      this(name, writeOutConnect, readInConnect, false, null);
   }

   public DataStreamYoWhiteBoard(String name, boolean writeOutConnect, boolean readInConnect, boolean createYoVariablesOnConnect, YoVariableRegistry rootRegistryToAddVariablesTo)
   {
      super(name, rootRegistryToAddVariablesTo);
      
      this.writeOutConnect = writeOutConnect;
      this.readInConnect = readInConnect;
      this.createYoVariablesOnConnect = createYoVariablesOnConnect;
      this.rootRegistryToAddVariablesTo = rootRegistryToAddVariablesTo;
      
      if (createYoVariablesOnConnect) 
      {
         if (rootRegistryToAddVariablesTo == null)
         {
            throw new RuntimeException("Trying to create YoVariables but not given a registry to create them with!");
         }
         
         if (!readInConnect)
         {
            throw new RuntimeException("Must read in connect if you wish to create the YoVariables to use.");
         }
      }
   }

   public void setDataStreams(DataInputStream dataInputStream, DataOutputStream dataOutputStream)
   {
      if (this.dataInputStream != null)
         throw new RuntimeException("dataInputStream != null");

      this.dataInputStream = dataInputStream;
      this.dataOutputStream = dataOutputStream;
   }

   public void run()
   {
      setupAndRunHandlingThread();
   }

   public void whiteBoardSpecificConnect() throws IOException
   {
      while (dataOutputStream == null)
      {
         try
         {
            Thread.sleep(10);
         }
         catch (InterruptedException e)
         {
         }
      }

      if (writeOutConnect)
      {
         if (VERBOSE)
            System.out.println("Writing out connect on the DataOutputStream");
         dataOutputStream.writeUTF("Connect");
         dataOutputStream.flush();

         while (!this.haveVariablesToReadAndWriteBeenSet())
         {
            Thread.yield();
         }
         
         ArrayList<YoVariable<?>> allVariablesToWrite = new ArrayList<YoVariable<?>>();
         this.getAllVariablesToWrite(allVariablesToWrite);

         writeVariableNamesToBeVerified(allVariablesToWrite);

         ArrayList<YoVariable<?>> allVariablesToRead = new ArrayList<YoVariable<?>>();
         this.getAllVariablesToRead(allVariablesToRead);

         writeVariableNamesToBeVerified(allVariablesToRead);
         
         if (VERBOSE)
            System.out.println("Writing out DoneConnect on the DataOutputStream");
         dataOutputStream.writeUTF("DoneConnect");
         dataOutputStream.flush();
      }
   }

   public void whiteBoardSpecificClose() throws IOException
   {
      setConnected(false);
      if (dataOutputStream != null)
         dataOutputStream.close();
      if (dataInputStream != null)
         dataInputStream.close();
   }

   private void writeVariableNamesToBeVerified(ArrayList<YoVariable<?>> variables) throws IOException
   {
      dataOutputStream.writeInt(variables.size());
      dataOutputStream.flush();

      for (YoVariable<?> yoVariable : variables)
      {
         dataOutputStream.writeUTF(yoVariable.getFullNameWithNameSpace());
         dataOutputStream.writeInt(yoVariable.getYoVariableType().ordinal());
         dataOutputStream.flush();
         
         try
         {
            Thread.sleep(1); // Give the other side some time to process it.
         } 
         catch (InterruptedException e)
         {
         }
      }
   }


   private void setupAndRunHandlingThread()
   {
      int tries = 0;
      while (!createYoVariablesOnConnect && !this.haveVariablesToReadAndWriteBeenSet())
      {
         try
         {
            Thread.sleep(10);
         }
         catch (InterruptedException e)
         {
         }

         tries++;
         if (tries % 1000 == 0)
            System.out.println("!haveVariablesToReadAndWriteBeenSet() after " + tries + " tries");
      }

      if (readInConnect)
      {
         try
         {
            if (VERBOSE)
               System.out.println("Reading for connect on the DataInputStream");
            String string = dataInputStream.readUTF();
            
            if (!string.equals("Connect"))
               throw new IOException("Didn't receive connect back!");

            if (createYoVariablesOnConnect)
            {
               ArrayList<YoVariable<?>> variablesToRead = this.readAndCreateVariables(rootRegistryToAddVariablesTo);
               this.setVariablesToRead(variablesToRead);

               ArrayList<YoVariable<?>> variablesToWrite = this.readAndCreateVariables(rootRegistryToAddVariablesTo);               
               this.setVariablesToWrite(variablesToWrite);
               
               if (VERBOSE)
                  System.out.println("DataStreamYoWhiteBoard: Created " + variablesToRead.size() + " variablesToRead and " + variablesToWrite.size() + " variablesToWrite");
            }
            
            else
            {
               ArrayList<YoVariable<?>> allVariablesToRead = new ArrayList<YoVariable<?>>();
               this.getAllVariablesToRead(allVariablesToRead);

               readAndVerifyVariableNames(allVariablesToRead);

               ArrayList<YoVariable<?>> allVariablesToWrite = new ArrayList<YoVariable<?>>();
               this.getAllVariablesToWrite(allVariablesToWrite);

               readAndVerifyVariableNames(allVariablesToWrite);
               
               if (VERBOSE)
                  System.out.println("DataStreamYoWhiteBoard: Read And Verified " + allVariablesToRead.size() + " variablesToRead and " + allVariablesToWrite.size() + " variablesToWrite");
            }

            if (VERBOSE)
               System.out.println("Reading for DoneConnect on the DataInputStream");
            string = dataInputStream.readUTF();

            if (!string.equals("DoneConnect"))
               throw new IOException("Didn't receive DoneConnect back!");

            if (VERBOSE)
               System.out.println("DataStreamYoWhiteBoard: Connected!");
            
         }
         catch (IOException e)
         {
            e.printStackTrace();

            return;
         }
      }

      if (!this.haveVariablesToReadAndWriteBeenSet())
      {
         throw new RuntimeException("!this.haveVariablesToReadAndWriteBeenSet(). How can that be?");
      }
      
      setConnected(true);
      allowThrowOutStalePacketsIfYouWish();

      double[] doubleVariablesToRead = new double[this.getNumberOfDoublesToRead()];
      int[] intVariablesToRead = new int[this.getNumberOfIntsToRead()];
      boolean[] booleanVariablesToRead = new boolean[this.getNumberOfBooleansToRead()];
      int[] enumVariablesToRead = new int[this.getNumberOfEnumsToRead()];

      while (isConnected())
      {
         try
         {
            int sync = dataInputStream.readInt();
            if (sync != TCPYoWhiteBoard.SYNC_IN)
               throw new IOException("sync != SYNC_IN");

            for (int i = 0; i < doubleVariablesToRead.length; i++)
            {
               doubleVariablesToRead[i] = dataInputStream.readDouble();
            }

            for (int i = 0; i < intVariablesToRead.length; i++)
            {
               intVariablesToRead[i] = dataInputStream.readInt();
            }

            for (int i = 0; i < booleanVariablesToRead.length; i++)
            {
               booleanVariablesToRead[i] = dataInputStream.readBoolean();
            }

            for (int i = 0; i < enumVariablesToRead.length; i++)
            {
               enumVariablesToRead[i] = dataInputStream.readInt();
            }

            int readIndex = dataInputStream.readInt();
            sync = dataInputStream.readInt();
            if (sync != DataStreamYoWhiteBoard.SYNC_OUT)
               throw new IOException("sync != SYNC_OUT");

            this.setVariablesToReadBuffers(doubleVariablesToRead, intVariablesToRead, booleanVariablesToRead, enumVariablesToRead, readIndex);
         }
         catch (java.net.SocketException socketException)
         {
            // Probably due to the socket being closed. Just exit peacefully.
            return;
         }
         catch (java.io.EOFException endOfFileException)
         {
            // Probably due to the socket being closed. Just exit peacefully.
            return;
         }
         catch (IOException e1)
         {
            e1.printStackTrace();

            return;
         }
      }
   }


   private void readAndVerifyVariableNames(ArrayList<YoVariable<?>> variables) throws IOException
   {
      int numberOfVariablesFromStream = dataInputStream.readInt();
      int numberOfVariables = variables.size();

      if (numberOfVariablesFromStream != numberOfVariables)
         throw new RuntimeException("numberOfVariablesFromStream = " + numberOfVariablesFromStream + ", but numberOfVariables = " + numberOfVariables);

      for (int i = 0; i < numberOfVariables; i++)
      {
         String fullName = dataInputStream.readUTF();
         int yoVariableTypeOrdinal = dataInputStream.readInt();
         YoVariableType yoVariableType = YoVariableType.values()[yoVariableTypeOrdinal];
         
         YoVariable<?> yoVariable = variables.get(i);
         if (yoVariable.getYoVariableType() != yoVariableType)
         {
            throw new RuntimeException("yoVariable.getYoVariableType() = " + yoVariable.getYoVariableType() + " != yoVariableType = " + yoVariableType);
         }
         verifyNamesAreConsistent(yoVariable.getFullNameWithNameSpace(), fullName);
      }

      if (VERBOSE)
         System.out.println("Found " + numberOfVariables + " variables and they all match names!");
   }
   
   
   private ArrayList<YoVariable<?>> readAndCreateVariables(YoVariableRegistry rootRegistry) throws IOException
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();
      
      int numberOfVariablesFromStream = dataInputStream.readInt();

      for (int i = 0; i < numberOfVariablesFromStream; i++)
      {
         String fullName = dataInputStream.readUTF();
         int yoVariableTypeOrdinal = dataInputStream.readInt();
         YoVariableType yoVariableType = YoVariableType.values()[yoVariableTypeOrdinal];
         
         String variableName = NameSpace.stripOffNameSpaceToGetVariableName(fullName);
         NameSpace fullNameSpace = NameSpace.createNameSpaceFromAFullVariableName(fullName);
         YoVariableRegistry registry = rootRegistry.getOrCreateAndAddRegistry(fullNameSpace);

         if (VERBOSE)
         {
            System.out.println("readAndCreateVariables: fullName = " + fullName);
         }
         
         YoVariable<?> yoVariable = createNewYoVariable(variableName, yoVariableType, registry);
         ret.add(yoVariable);
         
         verifyNamesAreConsistent(yoVariable.getFullNameWithNameSpace(), fullName);
      }

      if (VERBOSE)
         System.out.println("Found " + numberOfVariablesFromStream + " variable names and created them!");
      
      return ret;
   }

   private YoVariable<?> createNewYoVariable(String variableName, YoVariableType yoVariableType, YoVariableRegistry registry)
   {
      YoVariable<?> yoVariable;
      
      switch(yoVariableType)
      {
      case DOUBLE:
      {
         yoVariable = new DoubleYoVariable(variableName, registry);
         break;
      }
      
      case INTEGER:
      {
         yoVariable = new IntegerYoVariable(variableName, registry);
         break;
      }
      
      case BOOLEAN:
      {
         yoVariable = new BooleanYoVariable(variableName, registry);
         break;
      }
      
      default: 
      {
         // Treat EnumYoVariables as IntegerYoVariables if you are creating them here. Otherwise need to know the class itself...
         yoVariable = new IntegerYoVariable(variableName, registry);
      }
      }

      
      return yoVariable;
   }


   public void whiteBoardSpecificWriteData(double[] doubleVariablesToWriteBuffer, int[] intVariablesToWriteBuffer, boolean[] booleanVariablesToWriteBuffer,
           int[] enumVariablesToWriteBuffer, int writeIndex)
           throws IOException
   {
      dataOutputStream.writeInt(SYNC_IN);

      for (int i = 0; i < doubleVariablesToWriteBuffer.length; i++)
      {
         dataOutputStream.writeDouble(doubleVariablesToWriteBuffer[i]);
      }

      for (int i = 0; i < intVariablesToWriteBuffer.length; i++)
      {
         dataOutputStream.writeInt(intVariablesToWriteBuffer[i]);
      }

      for (int i = 0; i < booleanVariablesToWriteBuffer.length; i++)
      {
         dataOutputStream.writeBoolean(booleanVariablesToWriteBuffer[i]);
      }

      for (int i = 0; i < enumVariablesToWriteBuffer.length; i++)
      {
         dataOutputStream.writeInt(enumVariablesToWriteBuffer[i]);
      }

      dataOutputStream.writeInt(writeIndex);
      dataOutputStream.writeInt(SYNC_OUT);
      dataOutputStream.flush();

   }
}
