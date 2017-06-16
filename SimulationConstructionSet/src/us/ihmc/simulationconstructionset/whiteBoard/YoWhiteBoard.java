package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class YoWhiteBoard
{
   private static final boolean VERBOSE = false;
   
   private final String name;
   private final YoBoolean variablesToReadHaveBeenSet, variablesToWriteHaveBeenSet;

   private YoDouble[] doubleVariablesToRead;
   private IntegerYoVariable[] intVariablesToRead;
   private YoBoolean[] booleanVariablesToRead;
   private EnumYoVariable<?>[] enumVariablesToRead;

   private YoDouble[] doubleVariablesToWrite;
   private IntegerYoVariable[] intVariablesToWrite;
   private YoBoolean[] booleanVariablesToWrite;
   private EnumYoVariable<?>[] enumVariablesToWrite;

   private double[] doubleVariablesToReadBuffer;
   private int[] intVariablesToReadBuffer;
   private boolean[] booleanVariablesToReadBuffer;
   private int[] enumVariablesToReadBuffer;

   private double[] doubleVariablesToWriteBuffer;
   private int[] intVariablesToWriteBuffer;
   private boolean[] booleanVariablesToWriteBuffer;
   private int[] enumVariablesToWriteBuffer;

   
   private final IntegerYoVariable writeIndex, readIndex;
   private final IntegerYoVariable numberOfNewDataSinceLastRead;
   
   private ArrayList<YoWhiteBoardListener> yoWhiteBoardListeners;

   private boolean isConnected = false;

   public abstract void whiteBoardSpecificConnect() throws IOException;

   public abstract void closeYoWhiteBoard() throws IOException;

   public abstract void whiteBoardSpecificWriteData(double[] doubleVariablesToWriteBuffer, int[] intVariablesToWriteBuffer,
           boolean[] booleanVariablesToWriteBuffer, int[] enumVariablesToWriteBuffer, int writeIndex)
           throws IOException;

   public YoWhiteBoard(String name, YoVariableRegistry registry)
   {      
      this.name = name;
      this.variablesToReadHaveBeenSet = new YoBoolean(name + "VariablesToReadHaveBeenSet", registry);
      this.variablesToWriteHaveBeenSet = new YoBoolean(name + "VariableToWriteHaveBeenSet", registry);
      
      variablesToReadHaveBeenSet.set(false);
      variablesToWriteHaveBeenSet.set(false);
      
      writeIndex = new IntegerYoVariable(name + "WriteIndex", registry);
      readIndex = new IntegerYoVariable(name + "ReadIndex", registry);
      writeIndex.set(0);
      readIndex.set(0);
      
      numberOfNewDataSinceLastRead = new IntegerYoVariable(name + "NumberOfNewDataSinceLastRead", registry);
      numberOfNewDataSinceLastRead.set(0);
   }
   
   public synchronized final boolean isConnected()
   {
      return isConnected;
   }

   protected synchronized void setConnected(boolean isConnected)
   {
      this.isConnected = isConnected;
      if (isConnected)
         notifyAll();    // notify all threads that are waiting for this YoWhiteBoard to become connected
   }

   public int getNumberOfDoublesToRead()
   {
      return doubleVariablesToRead.length;
   }

   public int getNumberOfIntsToRead()
   {
      return intVariablesToRead.length;
   }

   public int getNumberOfBooleansToRead()
   {
      return booleanVariablesToRead.length;
   }

   public int getNumberOfEnumsToRead()
   {
      return enumVariablesToRead.length;
   }

   public synchronized boolean isNewDataAvailable()
   {
      return (numberOfNewDataSinceLastRead.getIntegerValue() > 0);
   }

   public synchronized int getNumberOfNewDataSinceLastRead()
   {
      return numberOfNewDataSinceLastRead.getIntegerValue();
   }

   public boolean haveVariablesToReadAndWriteBeenSet()
   {
      return (variablesToReadHaveBeenSet.getBooleanValue() && variablesToWriteHaveBeenSet.getBooleanValue());
   }

   public void connect() throws IOException
   {
      doubleVariablesToReadBuffer = new double[doubleVariablesToRead.length];
      intVariablesToReadBuffer = new int[intVariablesToRead.length];
      booleanVariablesToReadBuffer = new boolean[booleanVariablesToRead.length];
      enumVariablesToReadBuffer = new int[enumVariablesToRead.length];

      doubleVariablesToWriteBuffer = new double[doubleVariablesToWrite.length];
      intVariablesToWriteBuffer = new int[intVariablesToWrite.length];
      booleanVariablesToWriteBuffer = new boolean[booleanVariablesToWrite.length];
      enumVariablesToWriteBuffer = new int[enumVariablesToWrite.length];

      whiteBoardSpecificConnect();
   }

   public void writeData() throws IOException
   {
      if (!isConnected())
         throw new RuntimeException("Not yet connected");

      for (int i = 0; i < doubleVariablesToWrite.length; i++)
      {
         doubleVariablesToWriteBuffer[i] = doubleVariablesToWrite[i].getDoubleValue();
      }

      for (int i = 0; i < intVariablesToWrite.length; i++)
      {
         intVariablesToWriteBuffer[i] = intVariablesToWrite[i].getIntegerValue();
      }

      for (int i = 0; i < booleanVariablesToWrite.length; i++)
      {
         booleanVariablesToWriteBuffer[i] = booleanVariablesToWrite[i].getBooleanValue();
      }

      for (int i = 0; i < enumVariablesToWrite.length; i++)
      {
         Enum<?> enumValue = enumVariablesToWrite[i].getEnumValue();
         int ordinal = -1;

         if (enumValue != null)
         {
            ordinal = enumValue.ordinal();
         }

         enumVariablesToWriteBuffer[i] = ordinal;
      }

      whiteBoardSpecificWriteData(doubleVariablesToWriteBuffer, intVariablesToWriteBuffer, booleanVariablesToWriteBuffer, enumVariablesToWriteBuffer,
                                  writeIndex.getIntegerValue());
      writeIndex.increment();
   }
   
//   private void writeDataAgainOnRewind() throws IOException
//   {
//      whiteBoardSpecificWriteData(doubleVariablesToWriteBuffer, intVariablesToWriteBuffer, booleanVariablesToWriteBuffer, enumVariablesToWriteBuffer,
//            writeIndex.getIntegerValue());
//   }

   public synchronized void readData()
   {
      if (!isConnected())
         throw new RuntimeException("Not yet connected");

      for (int i = 0; i < doubleVariablesToRead.length; i++)
      {
         doubleVariablesToRead[i].set(doubleVariablesToReadBuffer[i]);
      }

      for (int i = 0; i < intVariablesToRead.length; i++)
      {
         intVariablesToRead[i].set(intVariablesToReadBuffer[i]);
      }

      for (int i = 0; i < booleanVariablesToRead.length; i++)
      {
         booleanVariablesToRead[i].set(booleanVariablesToReadBuffer[i]);
      }

      for (int i = 0; i < enumVariablesToRead.length; i++)
      {
         enumVariablesToRead[i].setValueFromDouble(enumVariablesToReadBuffer[i]);
      }

      numberOfNewDataSinceLastRead.set(0);
   }



   protected synchronized void setVariablesToReadBuffers(double[] doubleVariablesToReadBuffer, int[] intVariablesToReadBuffer,
           boolean[] booleanVariablesToReadBuffer, int[] enumVariablesToReadBuffer, int readIndex)
   {
      if (this.doubleVariablesToReadBuffer.length != doubleVariablesToReadBuffer.length)
         throw new RuntimeException("this.doubleVariablesToReadBuffer.length != doubleVariablesToReadBuffer.length");
      if (this.intVariablesToReadBuffer.length != intVariablesToReadBuffer.length)
         throw new RuntimeException("this.intVariablesToReadBuffer.length != intVariablesToReadBuffer.length");
      if (this.booleanVariablesToReadBuffer.length != booleanVariablesToReadBuffer.length)
         throw new RuntimeException("this.booleanVariablesToReadBuffer.length != booleanVariablesToReadBuffer.length");
      if (this.enumVariablesToReadBuffer.length != enumVariablesToReadBuffer.length)
         throw new RuntimeException("this.enumVariablesToReadBuffer.length != enumVariablesToReadBuffer.length");

      for (int i = 0; i < doubleVariablesToReadBuffer.length; i++)
      {
         this.doubleVariablesToReadBuffer[i] = doubleVariablesToReadBuffer[i];
      }

      for (int i = 0; i < intVariablesToReadBuffer.length; i++)
      {
         this.intVariablesToReadBuffer[i] = intVariablesToReadBuffer[i];
      }

      for (int i = 0; i < booleanVariablesToReadBuffer.length; i++)
      {
         this.booleanVariablesToReadBuffer[i] = booleanVariablesToReadBuffer[i];
      }

      for (int i = 0; i < enumVariablesToReadBuffer.length; i++)
      {
         this.enumVariablesToReadBuffer[i] = enumVariablesToReadBuffer[i];
      }

      numberOfNewDataSinceLastRead.increment();

      if (yoWhiteBoardListeners != null)
      {
         for (YoWhiteBoardListener listener : yoWhiteBoardListeners)
         {
            listener.receivedWhiteBoardData();
         }
      }

      this.readIndex.set(readIndex);
      notifyAll();    // wake up all threads that are waiting for new data from this YoWhiteBoard
   }

   public void attachYoWhiteBoardListener(YoWhiteBoardListener yoWhiteBoardListener)
   {
      if (yoWhiteBoardListeners == null)
      {
         yoWhiteBoardListeners = new ArrayList<YoWhiteBoardListener>();
      }

      yoWhiteBoardListeners.add(yoWhiteBoardListener);
   }

   public void getAllVariablesToWrite(ArrayList<YoVariable<?>> allVariablesToWrite)
   {
      for (YoVariable<?> yoVariable : doubleVariablesToWrite)
      {
         allVariablesToWrite.add(yoVariable);
      }

      for (YoVariable<?> yoVariable : intVariablesToWrite)
      {
         allVariablesToWrite.add(yoVariable);
      }

      for (YoVariable<?> yoVariable : booleanVariablesToWrite)
      {
         allVariablesToWrite.add(yoVariable);
      }

      for (YoVariable<?> yoVariable : enumVariablesToWrite)
      {
         allVariablesToWrite.add(yoVariable);
      }
   }

   public void getAllVariablesToRead(ArrayList<YoVariable<?>> allVariablesToRead)
   {
      for (YoVariable<?> yoVariable : doubleVariablesToRead)
      {
         allVariablesToRead.add(yoVariable);
      }

      for (YoVariable<?> yoVariable : intVariablesToRead)
      {
         allVariablesToRead.add(yoVariable);
      }

      for (YoVariable<?> yoVariable : booleanVariablesToRead)
      {
         allVariablesToRead.add(yoVariable);
      }

      for (YoVariable<?> yoVariable : enumVariablesToRead)
      {
         allVariablesToRead.add(yoVariable);
      }
   }

   public void setVariablesToWrite(ArrayList<YoVariable<?>> variablesToWrite)
   {
      ArrayList<YoDouble> doubleVariablesToWrite = new ArrayList<YoDouble>();
      ArrayList<IntegerYoVariable> intVariablesToWrite = new ArrayList<IntegerYoVariable>();
      ArrayList<YoBoolean> booleanVariablesToWrite = new ArrayList<YoBoolean>();
      ArrayList<EnumYoVariable<?>> enumVariablesToWrite = new ArrayList<EnumYoVariable<?>>();

      for (YoVariable<?> variableToWrite : variablesToWrite)
      {
         YoVariableType yoVariableType = variableToWrite.getYoVariableType();

         switch (yoVariableType)
         {
            case DOUBLE :
            {
               doubleVariablesToWrite.add((YoDouble) variableToWrite);

               break;
            }

            case INTEGER :
            {
               intVariablesToWrite.add((IntegerYoVariable) variableToWrite);

               break;
            }

            case BOOLEAN :
            {
               booleanVariablesToWrite.add((YoBoolean) variableToWrite);

               break;
            }

            case ENUM :
            {
               enumVariablesToWrite.add((EnumYoVariable<?>) variableToWrite);

               break;
            }

            default :
            {
               throw new RuntimeException("Shouldn't get here!");
            }
         }
      }

      this.doubleVariablesToWrite = new YoDouble[doubleVariablesToWrite.size()];
      this.intVariablesToWrite = new IntegerYoVariable[intVariablesToWrite.size()];
      this.booleanVariablesToWrite = new YoBoolean[booleanVariablesToWrite.size()];
      this.enumVariablesToWrite = new EnumYoVariable[enumVariablesToWrite.size()];

      doubleVariablesToWrite.toArray(this.doubleVariablesToWrite);
      intVariablesToWrite.toArray(this.intVariablesToWrite);
      booleanVariablesToWrite.toArray(this.booleanVariablesToWrite);
      enumVariablesToWrite.toArray(this.enumVariablesToWrite);

      if (VERBOSE)
      {
         System.out.println("Found " + this.doubleVariablesToWrite.length + " doubles.");
         System.out.println("Found " + this.intVariablesToWrite.length + " ints.");
         System.out.println("Found " + this.booleanVariablesToWrite.length + " booleans.");
         System.out.println("Found " + this.enumVariablesToWrite.length + " enums.");
      }

      variablesToWriteHaveBeenSet.set(true);
   }

   public void setVariablesToRead(ArrayList<YoVariable<?>> variablesToRead)
   {
      ArrayList<YoDouble> doubleVariablesToRead = new ArrayList<YoDouble>();
      ArrayList<IntegerYoVariable> intVariablesToRead = new ArrayList<IntegerYoVariable>();
      ArrayList<YoBoolean> booleanVariablesToRead = new ArrayList<YoBoolean>();
      ArrayList<EnumYoVariable<?>> enumVariablesToRead = new ArrayList<EnumYoVariable<?>>();

      for (YoVariable<?> variableToRead : variablesToRead)
      {
         YoVariableType yoVariableType = variableToRead.getYoVariableType();

         switch (yoVariableType)
         {
            case DOUBLE :
            {
               doubleVariablesToRead.add((YoDouble) variableToRead);

               break;
            }

            case INTEGER :
            {
               intVariablesToRead.add((IntegerYoVariable) variableToRead);

               break;
            }

            case BOOLEAN :
            {
               booleanVariablesToRead.add((YoBoolean) variableToRead);

               break;
            }

            case ENUM :
            {
               enumVariablesToRead.add((EnumYoVariable<?>) variableToRead);

               break;
            }

            default :
            {
               throw new RuntimeException("Shouldn't get here!");
            }
         }

      }

      this.doubleVariablesToRead = new YoDouble[doubleVariablesToRead.size()];
      this.intVariablesToRead = new IntegerYoVariable[intVariablesToRead.size()];
      this.booleanVariablesToRead = new YoBoolean[booleanVariablesToRead.size()];
      this.enumVariablesToRead = new EnumYoVariable[enumVariablesToRead.size()];

      doubleVariablesToRead.toArray(this.doubleVariablesToRead);
      intVariablesToRead.toArray(this.intVariablesToRead);
      booleanVariablesToRead.toArray(this.booleanVariablesToRead);
      enumVariablesToRead.toArray(this.enumVariablesToRead);

      variablesToReadHaveBeenSet.set(true);
   }

   protected void verifyNamesAreConsistent(String fullNameSpaceOne, String fullNameSpaceTwo)
   {
      final String variableNameOne = NameSpace.stripOffNameSpaceToGetVariableName(fullNameSpaceOne);
      final String variableNameTwo = NameSpace.stripOffNameSpaceToGetVariableName(fullNameSpaceTwo);
      if (!variableNameOne.equals(variableNameTwo))
      {
         throw new RuntimeException("Variable names are not the same. variableNameOne = \'" + variableNameOne + "\', variableNameTwo = \'" + variableNameTwo
                                    + "\'");
      }

//    if (!fullNameSpaceOne.endsWith(fullNameSpaceTwo))
//    {
//       if (!fullNameSpaceTwo.endsWith(fullNameSpaceOne))
//          throw new RuntimeException("Namespace endings not equal. fullNameSpaceOne = \'" + fullNameSpaceOne + "\', fullNameSpaceTwo = \'" + fullNameSpaceTwo + "\'");
//    }
   }

   public int getWriteIndex()
   {
      return writeIndex.getIntegerValue();
   }

   public int getReadIndex()
   {
      return readIndex.getIntegerValue();
   }

   public String getName()
   {
      return name;
   }
}
