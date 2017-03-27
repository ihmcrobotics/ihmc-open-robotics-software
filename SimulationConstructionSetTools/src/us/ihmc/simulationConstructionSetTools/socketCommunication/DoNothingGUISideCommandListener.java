package us.ihmc.simulationConstructionSetTools.socketCommunication;

import java.util.ArrayList;

public class DoNothingGUISideCommandListener implements GUISideAbstractCommandListener
{
   private final boolean VERBOSE = false;
   
   private final ArrayList<DoNothingGUISideCommandsReceived> commandsReceived = new ArrayList<DoNothingGUISideCommandsReceived>();


   public boolean hasReceivedSameCommands(DoNothingGUISideCommandListener doNothingGUISideCommandListener)
   {
      if (this.commandsReceived.size() != doNothingGUISideCommandListener.commandsReceived.size())
         return false;

      for (int i = 0; i < this.commandsReceived.size(); i++)
      {
         DoNothingGUISideCommandsReceived thisCommand = this.commandsReceived.get(i);
         DoNothingGUISideCommandsReceived thatCommand = doNothingGUISideCommandListener.commandsReceived.get(i);

         if (VERBOSE) System.out.println("DoNothingGUISideCommandListener.hasReceivedSameCommands: " + thisCommand);

         if (!thisCommand.areEqual(thatCommand)) 
         {
            if (VERBOSE) System.out.println("Commands are different! thisCommand = " + thisCommand + ", thatCommand = " + thatCommand);
            return false;
         }
      }

      return true;
   }

   @Override
   public void doHello(String name, String info)
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doHello() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.HELLO, name, info));
   }

   @Override
   public void doAllRegistriesAndVariables(String[] registryNames, String[][] variableNames, float[][] initialValues)
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doAllRegistriesAndVariables() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.ALL_REGISTRIES_AND_VARIABLES, registryNames, variableNames, initialValues));
   }

   @Override
   public void doRegistrySettingsProcessed(int[] registryIndices, boolean[] isSent, boolean[] isDisallowSendingSet, boolean[] isLogged, int registrySettingsIdentifier)
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doRegistrySettingsProcessed() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.REGISTRY_SETTINGS_PROCESSED, registrySettingsIdentifier));
   
    //TODO: Finish Me!!
      System.err.println("Warning! This doesn't check everything. Need to finish the implementation. We need to really clean this up!");
//      throw new RuntimeException("Finish Me!!"); 

   }

   @Override
   public void doSet(int index, float value)
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doSet() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.SET, index, value));
   }

   @Override
   public void doPeriod(int periodmsec)
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doPeriod() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.PERIOD, periodmsec));
   }

   @Override
   public void doUserCommand(String command)
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doUserCommand() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.USER_COMMAND, command));
   }

   @Override
   public void doData(float[] data)
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doData() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.DATA, data));
   }

   @Override
   public void doTextMessage(String message)
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doTextMessage() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.TEXT_MESSAGE, message));
   }

   @Override
   public void doDisconnect()
   {
      if (VERBOSE) System.out.println("DoNothingGUISideCommandListener doDisconnect() called.");
      commandsReceived.add(new DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum.DISCONNECT));
   }
   
   private class DoNothingGUISideCommandsReceived
   {
      private final DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum;
      private final Object argument1, argument2, argument3;
      private final String[] arrayOfStrings;
      private final String[][] doubleArrayOfStrings;
      private final float[] floatData;
      private final float[][] doubleArrayOfFloatData;
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, String[] strings, String[][] doubleArrayOfStrings, float[][] doubleArrayOfFloats)
      {
         this(doNothingGUISideCommandsReceivedEnum, null, null, null, strings, doubleArrayOfStrings,  null, doubleArrayOfFloats);
      }
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, String[] strings)
      {
         this(doNothingGUISideCommandsReceivedEnum, null, null, null, strings, null,  null, null);
      }
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, float[] floats)
      {
         this(doNothingGUISideCommandsReceivedEnum, null, null, null, null, null, floats, null);
      }
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, int integer, String[] strings, float[] floats)
      {
         this(doNothingGUISideCommandsReceivedEnum, integer, null, null, strings, null, floats, null);
      }
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, String stringOne)
      {
         this(doNothingGUISideCommandsReceivedEnum, stringOne, null, null, null, null, null, null);
      } 
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, String stringOne, String stringTwo)
      {
         this(doNothingGUISideCommandsReceivedEnum, null, null, null, new String[]{stringOne, stringTwo}, null, null, null);
      }  
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, int integer)
      {
         this(doNothingGUISideCommandsReceivedEnum, integer, null, null, null, null, null, null);
      } 
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, int integer, float floatValue)
      {
         this(doNothingGUISideCommandsReceivedEnum, integer, floatValue, null, null, null, null, null);
      } 
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum)
      {
         this(doNothingGUISideCommandsReceivedEnum, null, null, null, null, null, null, null);
      } 
      
      
      public DoNothingGUISideCommandsReceived(DoNothingGUISideCommandsReceivedEnum doNothingGUISideCommandsReceivedEnum, Object argument1, Object argument2, Object argument3, 
            String[] arrayOfStrings, String[][] doubleArrayOfStrings, float[] floatData, float[][] doubleArrayOfFloatData)
      {
         this.doNothingGUISideCommandsReceivedEnum = doNothingGUISideCommandsReceivedEnum;
         this.argument1 = argument1;
         this.argument2 = argument2;
         this.argument3 = argument3;
         
         this.arrayOfStrings = arrayOfStrings;
         this.doubleArrayOfStrings = doubleArrayOfStrings;
         this.floatData = floatData;
         this.doubleArrayOfFloatData = doubleArrayOfFloatData;
      }
      
      public boolean areEqual(DoNothingGUISideCommandsReceived doNothingGUISideCommandsReceived)
      {
         if (this.doNothingGUISideCommandsReceivedEnum != doNothingGUISideCommandsReceived.doNothingGUISideCommandsReceivedEnum) return false;
            
         if ((this.argument1 == null) && (doNothingGUISideCommandsReceived.argument1 != null)) return false;
         if ((this.argument2 == null) && (doNothingGUISideCommandsReceived.argument2 != null)) return false;
         if ((this.argument3 == null) && (doNothingGUISideCommandsReceived.argument3 != null)) return false;
         if ((this.arrayOfStrings == null) && (doNothingGUISideCommandsReceived.arrayOfStrings != null)) return false;
         if ((this.doubleArrayOfStrings == null) && (doNothingGUISideCommandsReceived.doubleArrayOfStrings != null)) return false;
         if ((this.floatData == null) && (doNothingGUISideCommandsReceived.floatData != null)) return false;
         if ((this.doubleArrayOfFloatData == null) && (doNothingGUISideCommandsReceived.doubleArrayOfFloatData != null)) return false;
         
         if ((this.argument1 != null) && (doNothingGUISideCommandsReceived.argument1 == null)) return false;
         if ((this.argument2 != null) && (doNothingGUISideCommandsReceived.argument2 == null)) return false;
         if ((this.argument3 != null) && (doNothingGUISideCommandsReceived.argument3 == null)) return false;
         if ((this.arrayOfStrings != null) && (doNothingGUISideCommandsReceived.arrayOfStrings == null)) return false;
         if ((this.doubleArrayOfStrings != null) && (doNothingGUISideCommandsReceived.doubleArrayOfStrings == null)) return false;
         if ((this.floatData != null) && (doNothingGUISideCommandsReceived.floatData == null)) return false;
         if ((this.doubleArrayOfFloatData != null) && (doNothingGUISideCommandsReceived.doubleArrayOfFloatData == null)) return false;

         if ((this.argument1 != null) && (!this.argument1.equals(doNothingGUISideCommandsReceived.argument1))) return false;
         if ((this.argument2 != null) && (!this.argument2.equals(doNothingGUISideCommandsReceived.argument2))) return false;
         if ((this.argument3 != null) && (!this.argument3.equals(doNothingGUISideCommandsReceived.argument3))) return false;
         if ((this.arrayOfStrings != null) && (!areArrayOfStringsEqual(this.arrayOfStrings, doNothingGUISideCommandsReceived.arrayOfStrings))) return false;
         if ((this.doubleArrayOfStrings != null) && (!areArrayOfStringsEqual(this.doubleArrayOfStrings, doNothingGUISideCommandsReceived.doubleArrayOfStrings))) return false;
         if ((this.floatData != null) && (!areArrayOfFloatsEqual(this.floatData, doNothingGUISideCommandsReceived.floatData))) return false;
         if ((this.doubleArrayOfFloatData != null) && (!areArrayOfFloatsEqual(this.doubleArrayOfFloatData, doNothingGUISideCommandsReceived.doubleArrayOfFloatData))) return false;
            
         return true;
      }
      
      private boolean areArrayOfStringsEqual(String[][] stringsOne, String[][] stringsTwo)
      {
         if (stringsOne.length != stringsTwo.length) return false;
         
         for (int i=0; i<stringsOne.length; i++)
         {
            if (!areArrayOfStringsEqual(stringsOne[i], stringsTwo[i])) return false;
         }
         
         return true;
      }
      
      private boolean areArrayOfStringsEqual(String[] stringsOne, String[] stringsTwo)
      {
         if (stringsOne.length != stringsTwo.length) 
         {
            if (VERBOSE) System.out.println("stringsOne.length = " + stringsOne.length + ", stringsTwo.length = " + stringsTwo.length);
            return false;
         }
         
         for (int i=0; i<stringsOne.length; i++)
         {
            if (!stringsOne[i].equals(stringsTwo[i]))
            {
               if (VERBOSE) System.out.println("stringsOne[i] = " + stringsOne[i] + ", stringsTwo[i] = " + stringsTwo[i]);
               return false;
            }
         }
         
         return true;
      }
      
      private boolean areArrayOfFloatsEqual(float[][] floatsOne, float[][] floatsTwo)
      {
         if (floatsOne.length != floatsTwo.length) return false;
         
         for (int i=0; i<floatsOne.length; i++)
         {
            if (!areArrayOfFloatsEqual(floatsOne[i], floatsTwo[i])) return false;
         }
         return true;
      }
      
      private boolean areArrayOfFloatsEqual(float[] floatsOne, float[] floatsTwo)
      {
         if (floatsOne.length != floatsTwo.length) return false;
         
         for (int i=0; i<floatsOne.length; i++)
         {
            if (Math.abs(floatsOne[i] - floatsTwo[i]) > 1e-4) return false;
         }
         
         return true;
      }
      
      @Override
      public String toString()
      {
         String ret = doNothingGUISideCommandsReceivedEnum + " argument1 = " + argument1 + ", argument2 = " + argument2 + ", argument3 = " + argument3 + ", arrayOfStrings = " + arrayOfStrings;
         
         return ret;
      }
   }

   
   private enum DoNothingGUISideCommandsReceivedEnum
   {
      HELLO, ALL_REGISTRIES_AND_VARIABLES, REGISTRY_SETTINGS_PROCESSED, SET, PERIOD, DISCONNECT, USER_COMMAND, DATA, TEXT_MESSAGE
   }




}
