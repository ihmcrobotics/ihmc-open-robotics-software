package us.ihmc.simulationconstructionset.util.dataProcessors;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.simulationconstructionset.DataProcessingFunction;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RobotAllJointsDataChecker implements DataProcessingFunction
{
   private double TOLERACE_FACTOR = 0.1;
   private HashMap<OneDegreeOfFreedomJoint, YoVariableValueDataChecker> listOfCheckers;
   
   
   
   public RobotAllJointsDataChecker(SimulationConstructionSet scs, Robot robot)
   {
      listOfCheckers = new HashMap<OneDegreeOfFreedomJoint, YoVariableValueDataChecker>();
      
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);
      for (OneDegreeOfFreedomJoint joint : oneDegreeOfFreedomJoints)
      {
         YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, joint.getQ(), robot.getYoTime());
         double upperLimit = joint.getJointUpperLimit();
         double lowerLimit = joint.getJointLowerLimit();
         double range = upperLimit - lowerLimit;
         
         if (Double.isNaN(range))
            throw new RuntimeException("upper joint limit - lower joint limit - NaN!");
         
         double limitAdjustment = range * TOLERACE_FACTOR;
         
         yoVariableValueDataChecker.setMaximumValue(upperLimit + limitAdjustment);
         yoVariableValueDataChecker.setMinimumValue(lowerLimit - limitAdjustment);

         yoVariableValueDataChecker.setMaximumDerivative((1.0 + TOLERACE_FACTOR)* joint.getVelocityLimit());
         
         //PDN: Joints don't have acceleration limits. Not sure what to do
         //yoVariableValueDataChecker.setMaximumSecondDerivate((1.0 * TOLERACE_FACTOR)* joint.get());
         
         listOfCheckers.put(joint, yoVariableValueDataChecker);
      }
   }

   @Override
   public void initializeProcessing()
   {
      for(YoVariableValueDataChecker checkers : listOfCheckers.values())
      {
         checkers.initializeProcessing();
      }
      
   }

   @Override
   public void processData()
   {
      for(YoVariableValueDataChecker checkers : listOfCheckers.values())
      {
         checkers.processData();
      }
      
   }
   
   public static void main(String[] args)
   {
      double a = Double.POSITIVE_INFINITY;
      double b = Double.NEGATIVE_INFINITY;
      
      double c = a-b;
      double e = a+b;
      double d = a + 10.0;
      
      System.out.println("b=" + b);
      System.out.println("c=" + c);
      System.out.println("d=" + d);
      System.out.println("e=" + e);
      
      System.out.println("is nan=" +    Double.isNaN(c));

      
   }
   
}
