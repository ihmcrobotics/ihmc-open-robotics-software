package us.ihmc.wholeBodyController.diagnostics;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.dataBuffer.DataProcessingFunction;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class HumanoidDiagnosticsWhenHangingAnalyzer
{
   private final SimulationConstructionSet simulationConstructionSet;
   private final DiagnosticsWhenHangingControllerState controller;
   private final FullRobotModelCorruptor fullRobotModelCorruptor;

   private final YoDouble q_qx, q_qy, q_qz, q_qs;
   private final YoDouble q_x, q_y, q_z;

   private final FullRobotModel fullRobotModel;

   private final ArrayList<YoVariable<?>> corruptorVariables;
   private final ArrayList<YoDouble> torqueOffsetVariables;
   private final ArrayList<YoDouble> torqueScoreVariables;

   private final ArrayList<YoVariable<?>> corruptorVariablesToOptimize= new ArrayList<YoVariable<?>>();
   private final ArrayList<YoVariable<YoDouble>>torqueScoresToOptimize= new ArrayList<YoVariable<YoDouble>>();

   private final ArrayList<OneDoFJoint> oneDoFJoints = new ArrayList<OneDoFJoint>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> jointsToJointAngles = new LinkedHashMap<OneDoFJoint, YoDouble>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> jointsToTorqueScore = new LinkedHashMap<OneDoFJoint, YoDouble>();
   private final YoDouble totalTorqueScore;

   private boolean stopOptimization;
   private final NumberFormat numberFormat;
   
   public HumanoidDiagnosticsWhenHangingAnalyzer(SimulationConstructionSet simulationConstructionSet,
           DiagnosticsWhenHangingControllerState controller, FullRobotModelCorruptor fullRobotModelCorruptor)
   {
      this.simulationConstructionSet = simulationConstructionSet;
      this.controller = controller;
      this.fullRobotModelCorruptor = fullRobotModelCorruptor;


      this.q_x = (YoDouble) simulationConstructionSet.getVariable("q_x");
      this.q_y = (YoDouble) simulationConstructionSet.getVariable("q_y");
      this.q_z = (YoDouble) simulationConstructionSet.getVariable("q_z");

      this.q_qx = (YoDouble) simulationConstructionSet.getVariable("q_qx");
      this.q_qy = (YoDouble) simulationConstructionSet.getVariable("q_qy");
      this.q_qz = (YoDouble) simulationConstructionSet.getVariable("q_qz");
      this.q_qs = (YoDouble) simulationConstructionSet.getVariable("q_qs");

      YoVariableRegistry corruptorRegistry = fullRobotModelCorruptor.getYoVariableRegistry();
      corruptorVariables = corruptorRegistry.getAllVariablesInThisListOnly();

      fullRobotModel = controller.getFullRobotModel();

      fullRobotModel.getOneDoFJoints(oneDoFJoints);

      YoVariableRegistry registry = new YoVariableRegistry("TorqueScore");
      simulationConstructionSet.addYoVariableRegistry(registry);

      torqueScoreVariables = new ArrayList<YoDouble>();
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         String jointAngleVariableName = "q_" + oneDoFJoint.getName();
         YoDouble jointAngleYoDouble = (YoDouble) simulationConstructionSet.getVariable(jointAngleVariableName);
         if (jointAngleYoDouble == null)
            throw new RuntimeException("Couldn't find variable " + jointAngleVariableName);
         jointsToJointAngles.put(oneDoFJoint, jointAngleYoDouble);

         YoDouble torqueScoreYoVariable = new YoDouble("score_" + oneDoFJoint.getName(), registry);

         jointsToTorqueScore.put(oneDoFJoint, torqueScoreYoVariable);
         torqueScoreVariables.add(torqueScoreYoVariable);
      }

      totalTorqueScore = new YoDouble("totalTorqueScore", registry);
      stopOptimization = false;
      
      torqueOffsetVariables = controller.getTorqueOffsetVariables();
      
      numberFormat = NumberFormat.getInstance();
      numberFormat.setMaximumFractionDigits(3);
      numberFormat.setMinimumFractionDigits(1);
      numberFormat.setGroupingUsed(false);
   }
   
   public void setVariablesToOptimize(String[] containsToOptimizeCoM, String[] containsToOptimizeTorqueScores)
   {
      corruptorVariablesToOptimize.clear();
      torqueScoresToOptimize.clear();

      for (YoVariable<?> variable : corruptorVariables)
      {
         for (String string : containsToOptimizeCoM)
         {
            if (variable.getName().contains(string))
            {
               corruptorVariablesToOptimize.add(variable);
               System.out.println("Adding " + variable.getName() + " to optimization variables.");
               break;
            }
         }
      } 

      for (YoDouble variable : torqueScoreVariables)
      {
         for (String string : containsToOptimizeTorqueScores)
         {
            if (variable.getName().contains(string))
            {
               torqueScoresToOptimize.add(variable);
               System.out.println("Adding " + variable.getName() + " to torqueScores to optimize.");
               break;
            }
         }
      }
   }
   
   public void copyMeasuredTorqueToAppliedTorque()
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         String measuredTorqueName = "raw_tau_" + oneDoFJoint.getName();
         YoDouble measuredTorque = (YoDouble) simulationConstructionSet.getVariable(measuredTorqueName);
         controller.setAppliedTorque(oneDoFJoint, measuredTorque.getDoubleValue());
      }
   }

   private double[] corruptorVariableValuesToRemember;

   public void rememberCorruptorVariableValues()
   {
      corruptorVariableValuesToRemember = new double[corruptorVariables.size()];
      for (int i=0; i<corruptorVariables.size(); i++)
      {
         corruptorVariableValuesToRemember[i] = corruptorVariables.get(i).getValueAsDouble();
      }
   }
   
   public void restoreCorruptorVariableValues()
   {
      for (int i=0; i<corruptorVariables.size(); i++)
      {
         corruptorVariables.get(i).setValueFromDouble(corruptorVariableValuesToRemember[i]);
      }
   }
   

   
   public void optimizeCorruptorValues(boolean computeTorqueOffsetsBasedOnAverages)
   {
      updateCorruptorAndAnalyzeDataInBuffer();
      simulationConstructionSet.gotoOutPointNow();

      double[] previousCorruptorVariableValues = new double[corruptorVariablesToOptimize.size()];
      LinkedHashMap<OneDoFJoint, Double> previousTorqueScoreValues = new LinkedHashMap<OneDoFJoint, Double>();
      
      double[] currentCorruptorVariableValues = new double[corruptorVariablesToOptimize.size()];
      LinkedHashMap<OneDoFJoint, Double> currentTorqueScoreValues = new LinkedHashMap<OneDoFJoint, Double>();

      getCurrentCorruptorValues(corruptorVariablesToOptimize, previousCorruptorVariableValues);
      resetTorqueScoreValuesToZero(previousTorqueScoreValues);
      
      double previousTotalTorqueScoreValue = totalTorqueScore.getDoubleValue();
      System.out.println("previousTotalTorqueScoreValue = " + previousTotalTorqueScoreValue);

      getCurrentCorruptorValues(corruptorVariablesToOptimize, currentCorruptorVariableValues);
      resetTorqueScoreValuesToZero(currentTorqueScoreValues);
      
      
//      int numberOfIterations = 250;
      double comOffsetChangeDelta = 0.01;
      
      Random random = new Random();
      
      while(true) // Stop by calling stopOptimization();
      { 
         if (stopOptimization)
         {
            stopOptimization = false;
            break;
         }

         moveCorruptorValuesRandomly(random, currentCorruptorVariableValues, comOffsetChangeDelta);
         setCorruptorValues(corruptorVariablesToOptimize, currentCorruptorVariableValues);
         
         this.updateDataAndComputeTorqueOffsetsBasedOnAverages(computeTorqueOffsetsBasedOnAverages);
         
         
         simulationConstructionSet.gotoOutPointNow();
         
         double newTotalTorqueScoreValue = totalTorqueScore.getDoubleValue();
//         System.out.println("newTotalTorqueScoreValue = " + newTotalTorqueScoreValue);
         
         if (newTotalTorqueScoreValue < previousTotalTorqueScoreValue)
         {
            // Improvement. Keep it.
            comOffsetChangeDelta = comOffsetChangeDelta * 1.2;
            if (comOffsetChangeDelta > 1.0) comOffsetChangeDelta = 1.0;
            System.out.println("\nImprovement. Keeping it! comOffsetChangeDelta increased to " + comOffsetChangeDelta);


            System.out.println("newTotalTorqueScoreValue = " + newTotalTorqueScoreValue);

            printOutAllCorruptorVariables();
            printOutCorruptorVariablesToOptimize();
            
            previousTotalTorqueScoreValue = newTotalTorqueScoreValue;

            getCurrentCorruptorValues(corruptorVariablesToOptimize, previousCorruptorVariableValues);
            //            resetTorqueScoreValuesToZero(previousTorqueScoreValues);

            simulationConstructionSet.repaintWindows();
         }
         else
         {
            // Not improvement. Don't keep it.

            comOffsetChangeDelta = comOffsetChangeDelta / 1.1;
            if (comOffsetChangeDelta < 0.001) comOffsetChangeDelta = 0.001;

            System.out.println("No improvement. Don't keep it! ChangeDelta decreased to " + comOffsetChangeDelta);

            setCurrentToPreviousValues(currentCorruptorVariableValues, previousCorruptorVariableValues);
            setCurrentToPreviousValues(currentTorqueScoreValues, previousTorqueScoreValues);

            setCorruptorValues(corruptorVariablesToOptimize, currentCorruptorVariableValues);
            //          updateCorruptorAndAnalyzeDataInBuffer();

            this.updateDataAndComputeTorqueOffsetsBasedOnAverages(computeTorqueOffsetsBasedOnAverages);
         }
      }
   }
   
   public void printOutCorruptorVariablesToOptimize()
   {
      System.out.println();
      for (YoVariable<?> yoVariable : corruptorVariablesToOptimize)
      {
         System.out.println(yoVariable.getName() + " = " + formatNumber(yoVariable) + ";");
      }
   }
   
   public void printOutAllCorruptorVariables()
   {
      System.out.println();
      for (YoVariable<?> yoVariable : corruptorVariables)
      {
         System.out.println(yoVariable.getName() + " = " + formatNumber(yoVariable) + ";");
      }
      
      System.out.println();
      for (YoVariable<?> yoVariable : torqueOffsetVariables)
      {
         System.out.println(yoVariable.getName() + " = " + formatNumber(yoVariable) + ";");
      }
   }
   
   private String formatNumber(YoVariable<?> variable)
   {
      return numberFormat.format(variable.getValueAsDouble());
   }

   public void stopOptimization()
   {
      stopOptimization = true;
   }
   
   private void moveCorruptorValuesRandomly(Random random, double[] currentCorruptorVariableValues, double changeDelta)
   {
      for (int i=0; i<currentCorruptorVariableValues.length; i++)
      {
         double corruptorValue = currentCorruptorVariableValues[i];
         
         double maxChange;
         
         if (Math.abs(corruptorValue) < 1.0)
         {
            maxChange = changeDelta;
         }
         else
         {
            maxChange = Math.abs(corruptorValue * changeDelta);
         }
         
         currentCorruptorVariableValues[i] = currentCorruptorVariableValues[i] + RandomNumbers.nextDouble(random, maxChange);
      }
   }


   private void setCurrentToPreviousValues(LinkedHashMap<OneDoFJoint, Double> currentValues,
         LinkedHashMap<OneDoFJoint, Double> previousValues)
   {
      currentValues.clear();
      
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         currentValues.put(oneDoFJoint, previousValues.get(oneDoFJoint));
      }
   }


   private void setCurrentToPreviousValues(double[] currentValues, double[] previousValues)
   {
      for (int i=0; i<previousValues.length; i++)
      {
         currentValues[i] = previousValues[i];
      }
   }


   private void getCurrentCorruptorValues(ArrayList<YoVariable<?>> corruptorVariables, double[] corruptorVariableValues)
   { 
      for (int i = 0; i < corruptorVariables.size(); i++)
      {
         YoVariable<?> variable = corruptorVariables.get(i);
         corruptorVariableValues[i] = variable.getValueAsDouble();
      }
   }
   
   private void getCurrentTorqueOffsetValues(ArrayList<YoDouble> torqueOffsetVariables, double[] torqueOffsetValues)
   { 
      for (int i = 0; i < torqueOffsetVariables.size(); i++)
      {
         YoDouble variable = torqueOffsetVariables.get(i);
         torqueOffsetValues[i] = variable.getDoubleValue();
      }
   } 
   
   public void setCorruptorVariableValuesToOptimizeToZero()
   {
      for (int i = 0; i < corruptorVariablesToOptimize.size(); i++)
      {
         YoVariable<?> variable = corruptorVariablesToOptimize.get(i);
         variable.setValueFromDouble(0.0);
      }
   }
   
   private void setCorruptorValues(ArrayList<YoVariable<?>> corruptorVariables, double[] corruptorVariableValues)
   { 
      for (int i = 0; i < corruptorVariables.size(); i++)
      {
         YoVariable<?> variable = corruptorVariables.get(i);
         variable.setValueFromDouble(corruptorVariableValues[i]);
      }
   }
   
   private void setTorqueOffsetValues(ArrayList<YoDouble> torqueOffsetVariables, double[] torqueOffsetValues)
   { 
      for (int i = 0; i < torqueOffsetVariables.size(); i++)
      {
         YoDouble variable = torqueOffsetVariables.get(i);
         variable.set(torqueOffsetValues[i]);
      }
   } 
   
   private void resetTorqueScoreValuesToZero(LinkedHashMap<OneDoFJoint, Double> torqueScoreValues)
   {
      torqueScoreValues.clear();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         torqueScoreValues.put(oneDoFJoint, 0.0);
      }
   }
   
   public void updateDataAndComputeTorqueOffsetsBasedOnAverages(boolean computeTorqueOffsetsBasedOnAverages)
   {
      updateCorruptorAndAnalyzeDataInBuffer();

      if (computeTorqueOffsetsBasedOnAverages)
      {
         computeTorqueOffsetsBasedOnAverages();
         updateCorruptorAndAnalyzeDataInBuffer();
      }
   }
   
   public void computeTorqueOffsetsBasedOnAverages()
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         YoDouble appliedTorque = controller.getAppliedTorqueYoVariable(oneDoFJoint);
         YoDouble estimatedTorque = controller.getEstimatedTorqueYoVariable(oneDoFJoint);
         YoDouble torqueOffset = controller.getTorqueOffsetVariable(oneDoFJoint);

         DataBuffer dataBuffer = simulationConstructionSet.getDataBuffer();
         
         if (appliedTorque != null)
         {
            double appliedTorqueAverage = dataBuffer.computeAverage(appliedTorque);
            double estimatedTorqueAverage = dataBuffer.computeAverage(estimatedTorque);

            torqueOffset.add(appliedTorqueAverage - estimatedTorqueAverage);

//            System.out.println("appliedTorque variable " + appliedTorque.getName() + " average = " + appliedTorqueAverage);
//            System.out.println("estimatedTorque variable " + estimatedTorque.getName() + " average = " + estimatedTorqueAverage);
//            System.out.println("Setting " + torqueOffset.getName() + " to " + torqueOffset.getDoubleValue());
         }
      }
   }
   
   public void updateCorruptorAndAnalyzeDataInBuffer()
   {
      DataProcessingFunction dataProcessingFunction = new DataProcessingFunction()
      {
         private final double[] corruptorVariableValues = new double[corruptorVariables.size()];
         private final double[] torqueOffsetValues = new double[torqueOffsetVariables.size()];
         private final LinkedHashMap<OneDoFJoint, Double> torqueScoreValues = new LinkedHashMap<OneDoFJoint, Double>();
         
         @Override
         public void initializeProcessing()
         {
            getCurrentCorruptorValues(corruptorVariables, corruptorVariableValues);
            getCurrentTorqueOffsetValues(torqueOffsetVariables, torqueOffsetValues);
            resetTorqueScoreValuesToZero(torqueScoreValues);
         }

         @Override
         public void processData()
         {
            FullRobotModel fullRobotModel = controller.getFullRobotModel();

            totalTorqueScore.set(0.0);

            for (OneDoFJoint oneDoFJoint : oneDoFJoints)
            {
               YoDouble jointAngleYoVariable = jointsToJointAngles.get(oneDoFJoint);
               oneDoFJoint.setQ(jointAngleYoVariable.getDoubleValue());
            }

            for (int i = 0; i < corruptorVariables.size(); i++)
            {
               YoVariable variable = corruptorVariables.get(i);
               variable.setValueFromDouble(corruptorVariableValues[i]);
            }

            for (int i = 0; i < torqueOffsetVariables.size(); i++)
            {
               YoDouble variable = torqueOffsetVariables.get(i);
               variable.set(torqueOffsetValues[i]);
            }

            fullRobotModelCorruptor.corruptFullRobotModel();

            RigidBodyTransform transform = new RigidBodyTransform();
            transform.setTranslation(new Vector3D(q_x.getDoubleValue(), q_y.getDoubleValue(), q_z.getDoubleValue()));
            transform.setRotation(new Quaternion(q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), q_qs.getDoubleValue()));

            fullRobotModel.getRootJoint().setPositionAndRotation(transform);
            fullRobotModel.updateFrames();

            controller.updateDiagnosticsWhenHangingHelpers();
            controller.addOffsetTorquesToAppliedTorques();


            for (OneDoFJoint oneDoFJoint : oneDoFJoints)
            {
               YoDouble torqueScoreYoVariable = jointsToTorqueScore.get(oneDoFJoint);
               Double torqueScoreValue = torqueScoreValues.get(oneDoFJoint);

               double appliedTorque = controller.getAppliedTorque(oneDoFJoint);
               double estimatedTorque = controller.getEstimatedTorque(oneDoFJoint);

               double squaredError = Math.pow(appliedTorque - estimatedTorque, 2.0);
               torqueScoreValues.put(oneDoFJoint, torqueScoreValue + squaredError);

               if (torqueScoresToOptimize.contains(torqueScoreYoVariable))
                  totalTorqueScore.add(torqueScoreValue);
               torqueScoreYoVariable.set(torqueScoreValue);
            }
         }

      };

      simulationConstructionSet.applyDataProcessingFunction(dataProcessingFunction);
   }

}

