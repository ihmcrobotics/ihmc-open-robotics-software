package us.ihmc.simulationconstructionset.util.simulationRunner;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.DataFileReader;

public class StateFileComparer
{
   /**
    * Compares two state files and returns a list of variables that differ by more than a given amount.
    * If variables differ, those variables' values are set to the absolute difference.
    *
    * @param filenameOne file name for first state file
    * @param filenameTwo file name for second state file
    * @param maxAbsoluteDiff max difference percentage between each YoVariable in state file 1 and state file 2
    * @param exceptions variables to skip while comparing state files
    * @return a VarList containing the YoVariables from state file one that differ more than maxAbsoluteDiff from those in state file 2
    */
   public static ArrayList<VariableDifference> absoluteCompareStateFiles(String filenameOne, String filenameTwo, double maxAbsoluteDiff, ArrayList<String> exceptions)
   {
      File fileOne = new File(filenameOne);
      File fileTwo = new File(filenameTwo);
      return absoluteCompareStateFiles(fileOne, fileTwo, maxAbsoluteDiff, exceptions);
   }


   /**
    * Compares two state files and returns a list of variables that differ by more than a given amount.
    * If variables differ, those variables' values are set to the absolute difference.
    *
    * @param fileOne file name for first state file
    * @param fileTwo file name for second state file
    * @param maxAbsoluteDiff max difference percentage between each YoVariable in state file 1 and state file 2
    * @param exceptions variables to skip while comparing state files
    * @return a VarList containing the YoVariables from state file one that differ more than maxAbsoluteDiff from those in state file 2
    */
   public static ArrayList<VariableDifference> absoluteCompareStateFiles(File fileOne, File fileTwo, double maxAbsoluteDiff, ArrayList<String> exceptions)
   {
      return compareStateFiles(fileOne, fileTwo, maxAbsoluteDiff, false, exceptions);
   }


   /**
    * Compares two state files and returns a list of variables that differ by more than a given percentage.
    * If variables differ, those variables' values are set to the percentage difference.
    *
    * @param filenameOne file name for first state file
    * @param filenameTwo file name for second state file
    * @param maxPercentDiff max difference percentage between each YoVariable in state file 1 and state file 2
    * @param exceptions variables to skip while comparing state files
    * @return a VarList containing the YoVariables from state file one that differ more than maxPercentDiff from those in state file 2
    */
   public static ArrayList<VariableDifference> percentualCompareStateFiles(String filenameOne, String filenameTwo, double maxPercentDiff, ArrayList<String> exceptions)
   {
      File fileOne = new File(filenameOne);
      File fileTwo = new File(filenameTwo);

      return percentualCompareStateFiles(fileOne, fileTwo, maxPercentDiff, exceptions);
   }


   /**
    * Compares two state files and returns a list of variables that differ by more than a given percentage.
    * If variables differ, those variables' values are set to the percentage difference.
    *
    * @param file1 first state file
    * @param file2 second state file
    * @param maxPercentDiff max difference percentage between each YoVariable in state file 1 and state file 2
    * @param exceptions variables to skip while comparing state files
    * @return a VarList containing the YoVariables from state file one that differ more than maxPercentDiff from those in state file 2
    */
   public static ArrayList<VariableDifference> percentualCompareStateFiles(File fileOne, File fileTwo, double maxPercentDiff, List<String> exceptions)
   {
      return compareStateFiles(fileOne, fileTwo, maxPercentDiff, true, exceptions);
   }
   

   private static ArrayList<VariableDifference> compareStateFiles(File fileOne, File fileTwo, double maxDifference, boolean checkForPercentDifference, List<String> exceptions)
   {
      DataFileReader dataFileReaderOne = new DataFileReader(fileOne);
      DataFileReader dataFileReaderTwo = new DataFileReader(fileTwo);

      YoVariableList varListOne = new YoVariableList("VarListOne");
      YoVariableList varListTwo = new YoVariableList("VarListTwo");

      YoVariableRegistry registryOne = new YoVariableRegistry("");
      YoVariableRegistry registryTwo = new YoVariableRegistry("");

      try
      {
         dataFileReaderOne.readState(varListOne, true, false, registryOne);
         dataFileReaderTwo.readState(varListTwo, true, false, registryTwo);
      }
      catch (Exception e)
      {
         e.printStackTrace();
         System.out.flush();
         System.err.flush();
      }

      return compareVarLists(varListOne, varListTwo, maxDifference, checkForPercentDifference, exceptions);
   }
   
   public static ArrayList<VariableDifference> compareVarLists(YoVariableList varListOne, YoVariableList varListTwo, double maxDifferenceAllowed, boolean checkForPercentDifference, List<String> exceptions)
   {
      VariablesThatShouldMatchList list = new VariablesThatShouldMatchList(varListOne, varListTwo, exceptions);
      ArrayList<VariableDifference> variableDifferences = new ArrayList<VariableDifference>();
      
      DoubleYoVariable timeYoVariable = (DoubleYoVariable) varListOne.getVariable("t");
      double time = Double.NaN;
      if (timeYoVariable != null)
      {
         time = timeYoVariable.getDoubleValue();
      }

      list.doVariableValuesMatch(variableDifferences, time, maxDifferenceAllowed, checkForPercentDifference);
      return variableDifferences;
   }


   public static void main(String[] args)
   {
//    String filenameOne = "Tests/test_2.2999.state";
//    String filenameTwo = "Tests/test_2.2999_Rewind.state";

      String filenameOne = "Tests/test_2.270000.state";
      String filenameTwo = "Tests/test_2.270000_Rewind.state";

//    String filenameOne = "090122_testAfter1_01Sec.state";
//    String filenameTwo = "090122_testAfter1_01Sec_Rewind.state";

//    String filenameOne = "090122_testAfter1_25Sec.state";
//    String filenameTwo = "090122_testAfter1_25Sec_Rewind.state";

//    String filenameOne = "090122_testAfter1_61Sec.state";
//    String filenameTwo = "090122_testAfter1_61Sec_Rewind.state";

//    String filenameOne = "090122_testAfter5_4Sec.state";
//    String filenameTwo = "090122_testAfter5_4Sec_Rewind.state";

//    String filenameOne = "090122_testAfter6Sec.state";
//    String filenameTwo = "090122_testAfter6Sec_Rewind.state";

      double maxPercentDiff = 0.05;

      ArrayList<VariableDifference> variableDifferences = StateFileComparer.percentualCompareStateFiles(filenameOne, filenameTwo, maxPercentDiff, null);

      for (VariableDifference variableDifference : variableDifferences)
      {
         System.out.println(variableDifference);
      }
   }
}
