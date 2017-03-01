package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.SocketException;
import java.util.ArrayList;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableType;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;


public abstract class DataStreamYoWhiteBoard extends YoWhiteBoard
{
   protected static final int SYNC_IN = 107;
   protected static final int SYNC_OUT = 393;
   
   private static final double CONNECTION_TIME_LIMIT = 5.0;
   
   private final Object connectionConch = new Object();
   private final Stopwatch connectionTimeoutTimer = new Stopwatch();

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

   @Override
   public void whiteBoardSpecificConnect() throws IOException
   {
      connectionTimeoutTimer.start();
      while (dataOutputStream == null)
      {
         if (connectionTimeoutTimer.lapElapsed() > CONNECTION_TIME_LIMIT)
         {
            throw new IOException("White board connect timed out after " + CONNECTION_TIME_LIMIT + " s");
         }
         
         ThreadTools.sleep(10);
      }

      if (writeOutConnect)
      {
         dataOutputStream.writeUTF("Connect");
         dataOutputStream.flush();

         connectionTimeoutTimer.lap();
         while (!this.haveVariablesToReadAndWriteBeenSet())
         {
            if (connectionTimeoutTimer.lapElapsed() > CONNECTION_TIME_LIMIT)
            {
               throw new IOException("White board connect timed out after " + CONNECTION_TIME_LIMIT + " s");
            }
             
            ThreadTools.sleep(100);
         }
         
         ArrayList<YoVariable<?>> allVariablesToWrite = new ArrayList<YoVariable<?>>();
         this.getAllVariablesToWrite(allVariablesToWrite);

         writeVariableNamesToBeVerified(allVariablesToWrite);

         ArrayList<YoVariable<?>> allVariablesToRead = new ArrayList<YoVariable<?>>();
         this.getAllVariablesToRead(allVariablesToRead);

         writeVariableNamesToBeVerified(allVariablesToRead);
         
         dataOutputStream.writeUTF("DoneConnect");
         dataOutputStream.flush();
      }
   }

   @Override
   public void closeYoWhiteBoard() throws IOException
   {
      setConnected(false);
      if (dataOutputStream != null)
      {
         try
         {
            dataOutputStream.close();
         }
         catch (SocketException socketException)
         {
            PrintTools.error(this, socketException.getMessage());
         }
         dataOutputStream = null;
      }
      if (dataInputStream != null)
      {
         try
         {
            dataInputStream.close();
         }
         catch (SocketException socketException)
         {
            PrintTools.error(this, socketException.getMessage());
         }
         dataInputStream = null;
      }
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


   protected void setupAndRunHandlingThread()
   {
      setupAndConnect();

      runHandlingThread();
   }

   protected void setupAndConnect()
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
            String string = dataInputStream.readUTF();
            
            if (!string.equals("Connect"))
               throw new IOException("Didn't receive connect back!");

            if (createYoVariablesOnConnect)
            {
               ArrayList<YoVariable<?>> variablesToRead = this.readAndCreateVariables(rootRegistryToAddVariablesTo);
               this.setVariablesToRead(variablesToRead);

               ArrayList<YoVariable<?>> variablesToWrite = this.readAndCreateVariables(rootRegistryToAddVariablesTo);               
               this.setVariablesToWrite(variablesToWrite);
            }
            
            else
            {
               ArrayList<YoVariable<?>> allVariablesToRead = new ArrayList<YoVariable<?>>();
               this.getAllVariablesToRead(allVariablesToRead);

               readAndVerifyVariableNames(allVariablesToRead);

               ArrayList<YoVariable<?>> allVariablesToWrite = new ArrayList<YoVariable<?>>();
               this.getAllVariablesToWrite(allVariablesToWrite);

               readAndVerifyVariableNames(allVariablesToWrite);
            }

            string = dataInputStream.readUTF();

            if (!string.equals("DoneConnect"))
               throw new IOException("Didn't receive DoneConnect back!");
         }
         catch (IOException e)
         {
            PrintTools.error(this, e.getMessage());

            return;
         }
      }

      if (!this.haveVariablesToReadAndWriteBeenSet())
      {
         throw new RuntimeException("!this.haveVariablesToReadAndWriteBeenSet(). How can that be?");
      }
      
      setConnected(true);
      allowThrowOutStalePacketsIfYouWish();
   }

   protected void runHandlingThread()
   {
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
            PrintTools.error(this, e1.getMessage());
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
         
         YoVariable<?> yoVariable = createNewYoVariable(variableName, yoVariableType, registry);
         ret.add(yoVariable);
         
         verifyNamesAreConsistent(yoVariable.getFullNameWithNameSpace(), fullName);
      }
      
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


   @Override
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

   public Object getConnectionConch()
   {
      return connectionConch;
   }
}
