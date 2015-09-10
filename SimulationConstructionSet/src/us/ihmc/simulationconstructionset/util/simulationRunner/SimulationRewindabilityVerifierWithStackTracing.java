package us.ihmc.simulationconstructionset.util.simulationRunner;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SimulationRewindabilityVerifierWithStackTracing
{
   private final double EPSILON = 1e-14;
   
   private final VariablesThatShouldMatchList VariablesThatShouldMatchList;
   private final YoVariableRegistry rootRegistryOne, rootRegistryTwo;
   private final LinkedHashMap<Thread, ArrayList<VariableChangeAndStackTrace>> changesForSimOne = new LinkedHashMap<Thread, ArrayList<VariableChangeAndStackTrace>>();
   private final LinkedHashMap<Thread, ArrayList<VariableChangeAndStackTrace>> changesForSimTwo = new LinkedHashMap<Thread, ArrayList<VariableChangeAndStackTrace>>();

   private boolean recordDifferencesForSimOne = false;
   private boolean recordDifferencesForSimTwo = false;

   public SimulationRewindabilityVerifierWithStackTracing(SimulationConstructionSet simulationOne, SimulationConstructionSet simulationTwo, ArrayList<String> exceptions)
   {
      rootRegistryOne = simulationOne.getRootRegistry();
      rootRegistryTwo = simulationTwo.getRootRegistry();
      VariablesThatShouldMatchList = new VariablesThatShouldMatchList(rootRegistryOne, rootRegistryTwo, exceptions);

      VariableChangedListener variableChangedListenerForSimOne = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> variableInRegistryOne)
         {
            if (!recordDifferencesForSimOne)
               return;

            YoVariable<?> variableInRegistryTwo = VariablesThatShouldMatchList.getYoVariableInListTwo(variableInRegistryOne.getFullNameWithNameSpace());
            StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();

            Thread thread = Thread.currentThread();
            VariableChangeAndStackTrace change = new VariableChangeAndStackTrace(thread, variableInRegistryOne, variableInRegistryTwo,
                                                    variableInRegistryOne.getValueAsDouble(), stackTrace);
            
            
            ArrayList<VariableChangeAndStackTrace> changesForThread = changesForSimOne.get(thread);
            if (changesForThread == null)
            {
               changesForThread = new ArrayList<SimulationRewindabilityVerifierWithStackTracing.VariableChangeAndStackTrace>();
               changesForSimOne.put(thread, changesForThread);
            }
            changesForThread.add(change);
         }
      };

      VariableChangedListener variableChangedListenerForSimTwo = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> variableInRegistryTwo)
         {
            if (!recordDifferencesForSimTwo)
               return;

            YoVariable<?> variableInRegistryOne = VariablesThatShouldMatchList.getYoVariableInListOne(variableInRegistryTwo.getFullNameWithNameSpace());
            StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
            
            Thread thread = Thread.currentThread();
            VariableChangeAndStackTrace change = new VariableChangeAndStackTrace(thread, variableInRegistryOne, variableInRegistryTwo,
                                                    variableInRegistryTwo.getValueAsDouble(), stackTrace);
            
            
            ArrayList<VariableChangeAndStackTrace> changesForThread = changesForSimTwo.get(thread);
            if (changesForThread == null)
            {
               changesForThread = new ArrayList<SimulationRewindabilityVerifierWithStackTracing.VariableChangeAndStackTrace>();
               changesForSimTwo.put(thread, changesForThread);
            }
            changesForThread.add(change);
         }
      };

      List<YoVariable<?>[]> variablesThatShouldMatch = VariablesThatShouldMatchList.getVariablesThatShouldMatch();
      for (YoVariable<?>[] variablesToMatch : variablesThatShouldMatch)
      {
         variablesToMatch[0].addVariableChangedListener(variableChangedListenerForSimOne);
         variablesToMatch[1].addVariableChangedListener(variableChangedListenerForSimTwo);
      }
   }

   public void clearChangesForSimulations()
   {
      changesForSimOne.clear();
      changesForSimTwo.clear();
   }
   
   public void setRecordDifferencesForSimOne(boolean recordDifferencesForSimOne)
   {
      this.recordDifferencesForSimOne = recordDifferencesForSimOne;
   }

   public void setRecordDifferencesForSimTwo(boolean recordDifferencesForSimTwo)
   {
      this.recordDifferencesForSimTwo = recordDifferencesForSimTwo;
   }

   public LinkedHashMap<Thread, ArrayList<VariableChangeAndStackTrace>> getVariableChangesAndStackTracesForSimulationOne()
   {
      return changesForSimOne;
   }

   public LinkedHashMap<Thread, ArrayList<VariableChangeAndStackTrace>> getVariableChangesAndStackTracesForSimulationTwo()
   {
      return changesForSimTwo;
   }

   public boolean areTheVariableChangesDifferent()
   {
      ArrayList<String> threadNames = getThreadNames();
      for (String threadName : threadNames)
      {
//         System.out.println("threadName = " + threadName);
         if (areTheVariableChangesDifferent(threadName)) return true;
      }
      
      return false;
   }
   
   public boolean areTheVariableChangesDifferent(String threadName)
   {
      Thread threadOne = getThreadWithName(threadName, this.changesForSimOne);
      Thread threadTwo = getThreadWithName(threadName, this.changesForSimTwo);
      
      ArrayList<VariableChangeAndStackTrace> changesForSimOne = this.changesForSimOne.get(threadOne);
      ArrayList<VariableChangeAndStackTrace> changesForSimTwo = this.changesForSimTwo.get(threadTwo);
      
      int sizeOne = changesForSimOne.size();
      int sizeTwo = changesForSimTwo.size();

      if (sizeOne != sizeTwo) return true;

      for (int i = 0; i < sizeOne; i++)
      {
         VariableChangeAndStackTrace changeOne = changesForSimOne.get(i);
         VariableChangeAndStackTrace changeTwo = changesForSimTwo.get(i);

         boolean variablesAreDifferent = !changeOne.variableOne.getName().equals(changeTwo.variableTwo.getName());
         boolean finalValueIsDifferent = Math.abs(changeOne.variableOne.getValueAsDouble() - changeOne.variableTwo.getValueAsDouble()) > EPSILON;
         boolean localChangeValuesAreDifferent = Math.abs(changeOne.variableValue - changeTwo.variableValue) > EPSILON;

         if (variablesAreDifferent || finalValueIsDifferent || localChangeValuesAreDifferent) return true;
      }

      return false;
   }
   
   private Thread getThreadWithName(String name, LinkedHashMap<Thread, ArrayList<VariableChangeAndStackTrace>> changes)
   {
      Set<Thread> keySet = changes.keySet();
      
      for (Thread thread : keySet)
      {
         if (thread.getName().equals(name)) return thread;
      }
      
      return null;
   }
   
   private ArrayList<String> getThreadNames()
   {
      ArrayList<String> threadNamesToReturn = new ArrayList<String>();

      Set<Thread> threadsForSimOne = changesForSimOne.keySet();
      
      for (Thread thread : threadsForSimOne)
      {
         threadNamesToReturn.add(thread.getName());
      }
      
      Set<Thread> threadsForSimTwo = changesForSimTwo.keySet();
      for (Thread thread : threadsForSimTwo)
      {
         if (!threadNamesToReturn.contains(thread.getName()))
         {
            throw new RuntimeException("Thread named" + thread.getName() + " is in second sim but not in first!");
         }
      }
      
      return threadNamesToReturn;
   }
   
   public int countNumberOfThreadsThatChangeTheVariables(ArrayList<VariableChangeAndStackTrace> changes)
   {      
      HashSet<Thread> threads = new HashSet<Thread>();
      
      for (int i = 0; i < changes.size(); i++)
      {
         VariableChangeAndStackTrace change = changes.get(i);

         Thread thread = change.thread;
         threads.add(thread);
      }

      return threads.size();
   }
   
   public int findIndexOfFistVariableChange(String threadName)
   {
      Thread threadOne = getThreadWithName(threadName, this.changesForSimOne);
      Thread threadTwo = getThreadWithName(threadName, this.changesForSimTwo);
      
      ArrayList<VariableChangeAndStackTrace> changesForSimOne = this.changesForSimOne.get(threadOne);
      ArrayList<VariableChangeAndStackTrace> changesForSimTwo = this.changesForSimTwo.get(threadTwo);
      
      int sizeOne = changesForSimOne.size();
      int sizeTwo = changesForSimTwo.size();

      int minSize = Math.min(sizeOne, sizeTwo);

      for (int index = 0; index < minSize; index++)
      {
         VariableChangeAndStackTrace changeOne = changesForSimOne.get(index);
         VariableChangeAndStackTrace changeTwo = changesForSimTwo.get(index);

         boolean variablesAreDifferent = !(changeOne.variableOne.getName().equals(changeTwo.variableTwo.getName()));
         boolean localChangeValuesAreDifferent = Math.abs(changeOne.variableValue - changeTwo.variableValue) > EPSILON;

         if (variablesAreDifferent || localChangeValuesAreDifferent) return index;
      }

      return -1;
   }
   
   public void printOutStackTracesOfFirstChangedVariable()
   {
      ArrayList<String> threadNames = getThreadNames();
      
      for (String threadName : threadNames)
      {
         System.out.println("\n\nThread = " + threadName);
         printOutStackTracesOfFirstChangedVariable(threadName);
      }
   }
   
   
   public void printOutStackTracesOfFirstChangedVariable(String threadName)
   {
      Thread threadOne = getThreadWithName(threadName, this.changesForSimOne);
      Thread threadTwo = getThreadWithName(threadName, this.changesForSimTwo);
      
      ArrayList<VariableChangeAndStackTrace> changesForSimOne = this.changesForSimOne.get(threadOne);
      ArrayList<VariableChangeAndStackTrace> changesForSimTwo = this.changesForSimTwo.get(threadTwo);
      
      int firstChangedIndex = findIndexOfFistVariableChange(threadName);
      if (firstChangedIndex < 0) return;
      System.out.println("\nFirst change index at : " + firstChangedIndex);

      System.out.println("\n Local changes for change one:");
      printLocalChangesBeforeAndAfterIndex(changesForSimOne, firstChangedIndex);
      System.out.println("\n Local changes for change two:");
      printLocalChangesBeforeAndAfterIndex(changesForSimTwo, firstChangedIndex);
      
      System.out.println("\n Stack traces for change one:");
      printStackTraceBeforeAndAfterIndex(changesForSimOne, firstChangedIndex);
      System.out.println("\n Stack traces for change two:");
      printStackTraceBeforeAndAfterIndex(changesForSimTwo, firstChangedIndex);
   }
   
   private static void printLocalChangesBeforeAndAfterIndex(ArrayList<VariableChangeAndStackTrace> changesForSim, int firstChangedIndex)
   {
      int sizeOne = changesForSim.size();

      int numberBefore = 2;
      int numberAfter = 2;
      
      int startIndex = Math.max(0, firstChangedIndex - numberBefore);
      int endIndex = Math.min(sizeOne - 1, firstChangedIndex + numberAfter);
      
      for (int i = startIndex; i <= endIndex; i++)
      {
         VariableChangeAndStackTrace change = changesForSim.get(i);
         String highlight = "";
         if (i == firstChangedIndex) highlight = "*** ";
         System.out.println(highlight + "Changed variable = " + change.variableOne.getName() + " = " + change.variableValue + ", other = " + change.variableTwo.getName());
      }
   }
   
   
   private static void printStackTraceBeforeAndAfterIndex(ArrayList<VariableChangeAndStackTrace> changesForSim, int firstChangedIndex)
   {
      int sizeOne = changesForSim.size();

      int numberBefore = 0;
      int numberAfter = 0;
      
      int startIndex = Math.max(0, firstChangedIndex - numberBefore);
      int endIndex = Math.min(sizeOne - 1, firstChangedIndex + numberAfter);
      
      for (int i = startIndex; i <= endIndex; i++)
      {
         VariableChangeAndStackTrace change = changesForSim.get(i);
         System.out.println("Changed variable = " + change.variableOne.getName() + " = " + change.variableValue + ", other = " + change.variableTwo.getName());
         printStackTrace(change.stackTrace);
      }
   }
   
   public void printOutVariableChangesStartingAtIndex(String threadName, int index)
   {
      Thread threadOne = getThreadWithName(threadName, this.changesForSimOne);
      Thread threadTwo = getThreadWithName(threadName, this.changesForSimTwo);
      
      ArrayList<VariableChangeAndStackTrace> changesForSimOne = this.changesForSimOne.get(threadOne);
      ArrayList<VariableChangeAndStackTrace> changesForSimTwo = this.changesForSimTwo.get(threadTwo);
      
      int sizeOne = changesForSimOne.size();
      int sizeTwo = changesForSimTwo.size();

      int minSize = Math.min(sizeOne, sizeTwo);

      for (int i = 0; i < minSize; i++)
      {
         VariableChangeAndStackTrace changeOne = changesForSimOne.get(i);
         VariableChangeAndStackTrace changeTwo = changesForSimTwo.get(i);
         
         System.out.println("Changed variable = " + changeOne.variableOne.getName() + " = " + changeOne.variableValue + ", other = " + changeTwo.variableTwo.getName() + " = " + changeTwo.variableValue);
      }
   }
   

   private static void printStackTrace(StackTraceElement[] stackTrace)
   {
      System.out.println("stack trace:");

      for (StackTraceElement stackTraceElement : stackTrace)
      {
         System.out.println(stackTraceElement.toString());
      }
   }


   private class VariableChangeAndStackTrace
   {
      private final Thread thread;
      private final YoVariable<?> variableOne;
      private final YoVariable<?> variableTwo;
      private final double variableValue;
      private final StackTraceElement[] stackTrace;

      public VariableChangeAndStackTrace(Thread thread, YoVariable<?> variableOne, YoVariable<?> variableTwo, double variableValue, StackTraceElement[] stackTrace)
      {
         this.thread = thread;
         this.variableOne = variableOne;
         this.variableTwo = variableTwo;
         this.variableValue = variableValue;
         this.stackTrace = stackTrace;
      }

   }

}
