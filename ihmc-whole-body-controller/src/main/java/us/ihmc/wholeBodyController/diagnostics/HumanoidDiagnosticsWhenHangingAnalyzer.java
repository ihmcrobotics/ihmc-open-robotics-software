package us.ihmc.wholeBodyController.diagnostics;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Random;

import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferProcessor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class HumanoidDiagnosticsWhenHangingAnalyzer
{
   private final SimulationConstructionSet simulationConstructionSet;
   private final DiagnosticsWhenHangingControllerState controller;
   private final FullRobotModelCorruptor fullRobotModelCorruptor;

   private final YoDouble q_qx, q_qy, q_qz, q_qs;
   private final YoDouble q_x, q_y, q_z;

   private final FullRobotModel fullRobotModel;

   private final List<YoVariable> corruptorVariables;
   private final ArrayList<YoDouble> torqueOffsetVariables;
   private final ArrayList<YoDouble> torqueScoreVariables;

   private final ArrayList<YoVariable> corruptorVariablesToOptimize = new ArrayList<>();
   private final ArrayList<YoDouble> torqueScoresToOptimize = new ArrayList<>();

   private final ArrayList<OneDoFJointBasics> oneDoFJoints = new ArrayList<OneDoFJointBasics>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> jointsToJointAngles = new LinkedHashMap<OneDoFJointBasics, YoDouble>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> jointsToTorqueScore = new LinkedHashMap<OneDoFJointBasics, YoDouble>();
   private final YoDouble totalTorqueScore;

   private boolean stopOptimization;
   private final NumberFormat numberFormat;

   public HumanoidDiagnosticsWhenHangingAnalyzer(SimulationConstructionSet simulationConstructionSet, DiagnosticsWhenHangingControllerState controller,
                                                 FullRobotModelCorruptor fullRobotModelCorruptor)
   {
      this.simulationConstructionSet = simulationConstructionSet;
      this.controller = controller;
      this.fullRobotModelCorruptor = fullRobotModelCorruptor;

      this.q_x = (YoDouble) simulationConstructionSet.findVariable("q_x");
      this.q_y = (YoDouble) simulationConstructionSet.findVariable("q_y");
      this.q_z = (YoDouble) simulationConstructionSet.findVariable("q_z");

      this.q_qx = (YoDouble) simulationConstructionSet.findVariable("q_qx");
      this.q_qy = (YoDouble) simulationConstructionSet.findVariable("q_qy");
      this.q_qz = (YoDouble) simulationConstructionSet.findVariable("q_qz");
      this.q_qs = (YoDouble) simulationConstructionSet.findVariable("q_qs");

      YoRegistry corruptorRegistry = fullRobotModelCorruptor.getYoVariableRegistry();
      corruptorVariables = corruptorRegistry.getVariables();

      fullRobotModel = controller.getFullRobotModel();

      fullRobotModel.getOneDoFJoints(oneDoFJoints);

      YoRegistry registry = new YoRegistry("TorqueScore");
      simulationConstructionSet.addYoRegistry(registry);

      torqueScoreVariables = new ArrayList<YoDouble>();
      for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
      {
         String jointAngleVariableName = "q_" + oneDoFJoint.getName();
         YoDouble jointAngleYoDouble = (YoDouble) simulationConstructionSet.findVariable(jointAngleVariableName);
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

      for (YoVariable variable : corruptorVariables)
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
      for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
      {
         String measuredTorqueName = "raw_tau_" + oneDoFJoint.getName();
         YoDouble measuredTorque = (YoDouble) simulationConstructionSet.findVariable(measuredTorqueName);
         controller.setAppliedTorque(oneDoFJoint, measuredTorque.getDoubleValue());
      }
   }

   private double[] corruptorVariableValuesToRemember;

   public void rememberCorruptorVariableValues()
   {
      corruptorVariableValuesToRemember = new double[corruptorVariables.size()];
      for (int i = 0; i < corruptorVariables.size(); i++)
      {
         corruptorVariableValuesToRemember[i] = corruptorVariables.get(i).getValueAsDouble();
      }
   }

   public void restoreCorruptorVariableValues()
   {
      for (int i = 0; i < corruptorVariables.size(); i++)
      {
         corruptorVariables.get(i).setValueFromDouble(corruptorVariableValuesToRemember[i]);
      }
   }

   public void optimizeCorruptorValues(boolean computeTorqueOffsetsBasedOnAverages)
   {
      updateCorruptorAndAnalyzeDataInBuffer();
      simulationConstructionSet.gotoOutPointNow();

      double[] previousCorruptorVariableValues = new double[corruptorVariablesToOptimize.size()];
      LinkedHashMap<OneDoFJointBasics, Double> previousTorqueScoreValues = new LinkedHashMap<OneDoFJointBasics, Double>();

      double[] currentCorruptorVariableValues = new double[corruptorVariablesToOptimize.size()];
      LinkedHashMap<OneDoFJointBasics, Double> currentTorqueScoreValues = new LinkedHashMap<OneDoFJointBasics, Double>();

      getCurrentCorruptorValues(corruptorVariablesToOptimize, previousCorruptorVariableValues);
      resetTorqueScoreValuesToZero(previousTorqueScoreValues);

      double previousTotalTorqueScoreValue = totalTorqueScore.getDoubleValue();
      System.out.println("previousTotalTorqueScoreValue = " + previousTotalTorqueScoreValue);

      getCurrentCorruptorValues(corruptorVariablesToOptimize, currentCorruptorVariableValues);
      resetTorqueScoreValuesToZero(currentTorqueScoreValues);

      //      int numberOfIterations = 250;
      double comOffsetChangeDelta = 0.01;

      Random random = new Random();

      while (true) // Stop by calling stopOptimization();
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
            if (comOffsetChangeDelta > 1.0)
               comOffsetChangeDelta = 1.0;
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
            if (comOffsetChangeDelta < 0.001)
               comOffsetChangeDelta = 0.001;

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
      for (YoVariable yoVariable : corruptorVariablesToOptimize)
      {
         System.out.println(yoVariable.getName() + " = " + formatNumber(yoVariable) + ";");
      }
   }

   public void printOutAllCorruptorVariables()
   {
      System.out.println();
      for (YoVariable yoVariable : corruptorVariables)
      {
         System.out.println(yoVariable.getName() + " = " + formatNumber(yoVariable) + ";");
      }

      System.out.println();
      for (YoVariable yoVariable : torqueOffsetVariables)
      {
         System.out.println(yoVariable.getName() + " = " + formatNumber(yoVariable) + ";");
      }
   }

   private String formatNumber(YoVariable variable)
   {
      return numberFormat.format(variable.getValueAsDouble());
   }

   public void stopOptimization()
   {
      stopOptimization = true;
   }

   private void moveCorruptorValuesRandomly(Random random, double[] currentCorruptorVariableValues, double changeDelta)
   {
      for (int i = 0; i < currentCorruptorVariableValues.length; i++)
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

   private void setCurrentToPreviousValues(LinkedHashMap<OneDoFJointBasics, Double> currentValues, LinkedHashMap<OneDoFJointBasics, Double> previousValues)
   {
      currentValues.clear();

      for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
      {
         currentValues.put(oneDoFJoint, previousValues.get(oneDoFJoint));
      }
   }

   private void setCurrentToPreviousValues(double[] currentValues, double[] previousValues)
   {
      for (int i = 0; i < previousValues.length; i++)
      {
         currentValues[i] = previousValues[i];
      }
   }

   private void getCurrentCorruptorValues(List<YoVariable> corruptorVariables, double[] corruptorVariableValues)
   {
      for (int i = 0; i < corruptorVariables.size(); i++)
      {
         YoVariable variable = corruptorVariables.get(i);
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
         YoVariable variable = corruptorVariablesToOptimize.get(i);
         variable.setValueFromDouble(0.0);
      }
   }

   private void setCorruptorValues(ArrayList<YoVariable> corruptorVariables, double[] corruptorVariableValues)
   {
      for (int i = 0; i < corruptorVariables.size(); i++)
      {
         YoVariable variable = corruptorVariables.get(i);
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

   private void resetTorqueScoreValuesToZero(LinkedHashMap<OneDoFJointBasics, Double> torqueScoreValues)
   {
      torqueScoreValues.clear();

      for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
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
      for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
      {
         YoDouble appliedTorque = controller.getAppliedTorqueYoVariable(oneDoFJoint);
         YoDouble estimatedTorque = controller.getEstimatedTorqueYoVariable(oneDoFJoint);
         YoDouble torqueOffset = controller.getTorqueOffsetVariable(oneDoFJoint);

         YoBuffer dataBuffer = simulationConstructionSet.getDataBuffer();

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
      YoBufferProcessor dataProcessingFunction = new YoBufferProcessor()
      {
         private final double[] corruptorVariableValues = new double[corruptorVariables.size()];
         private final double[] torqueOffsetValues = new double[torqueOffsetVariables.size()];
         private final LinkedHashMap<OneDoFJointBasics, Double> torqueScoreValues = new LinkedHashMap<OneDoFJointBasics, Double>();

         @Override
         public void initialize(YoVariableHolder yoVariableHolder)
         {
            getCurrentCorruptorValues(corruptorVariables, corruptorVariableValues);
            getCurrentTorqueOffsetValues(torqueOffsetVariables, torqueOffsetValues);
            resetTorqueScoreValuesToZero(torqueScoreValues);
         }

         @Override
         public void process(int startIndex, int endIndex, int currentIndex)
         {
            FullRobotModel fullRobotModel = controller.getFullRobotModel();

            totalTorqueScore.set(0.0);

            for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
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
            transform.getTranslation().set(new Vector3D(q_x.getDoubleValue(), q_y.getDoubleValue(), q_z.getDoubleValue()));
            transform.getRotation().set(new Quaternion(q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), q_qs.getDoubleValue()));

            fullRobotModel.getRootJoint().setJointConfiguration(transform);
            fullRobotModel.updateFrames();

            controller.updateDiagnosticsWhenHangingHelpers();
            controller.addOffsetTorquesToAppliedTorques();

            for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
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
