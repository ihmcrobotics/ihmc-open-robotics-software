package us.ihmc.simulationconstructionsettools.util.dataProcessors;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.simulationconstructionset.DataProcessingFunction;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RobotAllJointsDataChecker implements DataProcessingFunction
{
   private double TOLERANCE_FACTOR = 0.1;
   private double MINIMUM_TIME_TO_ACCLERATE = 1.0/20.0;
   private double DEFAULT_VELOCITY_ERROR_THRESHOLD = 0.5; //Arbitrarily pick this
   private HashMap<OneDegreeOfFreedomJoint, YoVariableValueDataChecker> listOfCheckers;
   
   private final SimulationConstructionSet scs;

   public RobotAllJointsDataChecker(SimulationConstructionSet scs, Robot robot)
   {
      this(scs, robot, getAllRobotJoints(robot));
   }
   
   public RobotAllJointsDataChecker(SimulationConstructionSet scs, Robot robot, ArrayList<OneDegreeOfFreedomJoint> jointsToCheck)
   {
      this.scs = scs;
      listOfCheckers = new HashMap<OneDegreeOfFreedomJoint, YoVariableValueDataChecker>();
      
      for (OneDegreeOfFreedomJoint joint : jointsToCheck)
      {
         ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
         
         double upperLimit = joint.getJointUpperLimit();
         double lowerLimit = joint.getJointLowerLimit();
         double range = upperLimit - lowerLimit;
         
         if (Double.isNaN(range))
            throw new RuntimeException("upper joint limit - lower joint limit - NaN!");
         
         if (range == 0.0)
            throw new RuntimeException("upper joint limit = lower joint limit!");
         
         double limitAdjustment = range * TOLERANCE_FACTOR;
         
         valueDataCheckerParameters.setMaximumValue(upperLimit + limitAdjustment);
         valueDataCheckerParameters.setMinimumValue(lowerLimit - limitAdjustment);

         valueDataCheckerParameters.setMaximumDerivative((1.0 + TOLERANCE_FACTOR)* joint.getVelocityLimit());
         
         valueDataCheckerParameters.setMaximumSecondDerivative(joint.getVelocityLimit()/ MINIMUM_TIME_TO_ACCLERATE);
         
         valueDataCheckerParameters.setErrorThresholdOnDerivativeComparison(DEFAULT_VELOCITY_ERROR_THRESHOLD);
         
         
         YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, joint.getQYoVariable(), robot.getYoTime(), valueDataCheckerParameters, joint.getQDYoVariable());
         
         //PDN: Joints don't have acceleration limits. Not sure what to do
         //yoVariableValueDataChecker.setMaximumSecondDerivate((1.0 * TOLERACE_FACTOR)* joint.get());
         
         listOfCheckers.put(joint, yoVariableValueDataChecker);
      }
   }
   
   private static ArrayList<OneDegreeOfFreedomJoint> getAllRobotJoints(Robot robot)
   {
      ArrayList<OneDegreeOfFreedomJoint> ret = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(ret);
      return ret;
   }
   
   public void setMaximumDerivativeForAllJoints(double maximumDerivative)
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         setMaximumDerivativeForJoint(joint, maximumDerivative);
      }
   }
   
   
   public void setMaximumDerivativeForJoint(OneDegreeOfFreedomJoint joint, double maximumDerivative)
   {
      listOfCheckers.get(joint).setMaximumDerivative(maximumDerivative);
   }
   
   public void setMaximumSecondDerivativeForAllJoints(double maximumSecondDerivative)
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         setMaximumSecondDerivativeForJoint(joint, maximumSecondDerivative);
      }
   }
   
   public void setMaximumSecondDerivativeForJoint(OneDegreeOfFreedomJoint joint, double maximumSecondDerivative)
   {
      listOfCheckers.get(joint).setMaximumSecondDerivate(maximumSecondDerivative);
   }
   
   public boolean isDerivativeComparisonErrorOccurred(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).isDerivativeCompErrorOccurred();
   }

   public boolean isMaxDerivativeExeeded(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).isMaxDerivativeExeeded();
   }

   public boolean isMaxSecondDerivativeExeeded(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).isMaxSecondDerivativeExeeded();
   }

   public boolean isMaxValueExeeded(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).isMaxValueExeeded();
   }

   public boolean isMinValueExeeded(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).isMinValueExeeded();
   }
   
   public boolean hasDerivativeComparisonErrorOccurredAnyJoint()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isDerivativeComparisonErrorOccurred(joint))
            return true;
      }
      return false;
   }

   public boolean hasMaxDerivativeExeededAnyJoint()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isMaxDerivativeExeeded(joint))
            return true;
      }
      return false;
   }

   public boolean hasMaxSecondDerivativeExeededAnyJoint()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isMaxSecondDerivativeExeeded(joint))
            return true;
      }
      return false;
   }

   public boolean hasMaxValueExeededAnyJoint()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isMaxValueExeeded(joint))
            return true;
      }
      return false;
   }

   public boolean hasMinValueExeededAnyJoint()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isMinValueExeeded(joint))
            return true;
      }
      return false;
   }
   
   public OneDegreeOfFreedomJoint getJointWhichHasDerivativeComparisonError()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isDerivativeComparisonErrorOccurred(joint))
            return joint;
      }
      return null;
   }

   public OneDegreeOfFreedomJoint getJointWhichExceededMaxDerivative()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isMaxDerivativeExeeded(joint))
            return joint;
      }
      return null;
   }

   public OneDegreeOfFreedomJoint getJointWhichExceededMaxSecondDerivative()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isMaxSecondDerivativeExeeded(joint))
            return joint;
      }
      return null;
   }

   public OneDegreeOfFreedomJoint getJointWhichExceededMaxValue()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isMaxValueExeeded(joint))
            return joint;
      }
      return null;
   }

   public OneDegreeOfFreedomJoint getJointWhichExceededMinValue()
   {
      for(OneDegreeOfFreedomJoint joint : listOfCheckers.keySet())
      {
         if (isMinValueExeeded(joint))
            return joint;
      }
      return null;
   }
   
   public double getMaxValueOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getMaxValue();
   }
   
   public double getMinValueOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getMinValue();
   }
   
   public double getMaxDerivativeOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getMaxDerivative();
   }
   
   public double getMaxSecondDerivativeOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getMaxSecondDerivative();
   }
   
   public double getSimTimeMaxValueOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getMaxValueSimTime();
   }
   
   public double getSimTimeMinValueOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getMinValueSimTime();
   }
   
   public double getSimTimeMaxDerivativeOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getMaxDerivativeSimTime();
   }
   
   public double getSimTimeMaxSecondDerivativeOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getMaxSecondDerivativeSimTime();
   }
   
   public double getSimTimeDeriveCompErrorOfJoint(OneDegreeOfFreedomJoint joint)
   {
      return listOfCheckers.get(joint).getDerivativeCompErrorSimTime();
   }
   
   public String getDerivativeCompError()
   {
      if (hasDerivativeComparisonErrorOccurredAnyJoint())
      {
         OneDegreeOfFreedomJoint failedJoint = getJointWhichHasDerivativeComparisonError();
         return failedJoint.getName() + " experienced a derivative computation error / discontinuity at t = " + getSimTimeDeriveCompErrorOfJoint(failedJoint) +  " seconds.";
      }
      return "";
   }

   public String getMaxDerivativeExceededError()
   {
      if (hasMaxDerivativeExeededAnyJoint())
      {
         OneDegreeOfFreedomJoint failedJoint = getJointWhichExceededMaxDerivative();
         return failedJoint.getName() + " reached " + getMaxDerivativeOfJoint(failedJoint) + " radians/sec, at t = "
               + getSimTimeMaxDerivativeOfJoint(failedJoint) + ", which exceeds the maximum joint velocity.";
      }
      return "";
   }

   public String getMaxSecondDerivativeExceededError()
   {
      if (hasMaxSecondDerivativeExeededAnyJoint())
      {
         OneDegreeOfFreedomJoint failedJoint = getJointWhichExceededMaxSecondDerivative();
         return failedJoint.getName() + " reached " + getMaxSecondDerivativeOfJoint(failedJoint) + " radians/sec/sec, at t = "
               + getSimTimeMaxSecondDerivativeOfJoint(failedJoint) + ", which exceeds the maximum joint acceleration.";
      }
      return "";
   }

   public String getMaxValueExceededError()
   {
      if (hasMaxValueExeededAnyJoint())
      {
         OneDegreeOfFreedomJoint failedJoint = getJointWhichExceededMaxValue();
         return failedJoint.getName() + " reached " + getMaxValueOfJoint(failedJoint) + " radians, at t = " + getSimTimeMaxValueOfJoint(failedJoint)
               + ", which is outside the allowable range of [" + failedJoint.getJointLowerLimit() + ", " + failedJoint.getJointUpperLimit() + "]";
      }
      return "";
   }

   public String getMinValueExceededError()
   {
      if (hasMinValueExeededAnyJoint())
      {
         OneDegreeOfFreedomJoint failedJoint = getJointWhichExceededMinValue();
         return failedJoint.getName() + " reached " + getMinValueOfJoint(failedJoint) + " radians, at t = " + getSimTimeMinValueOfJoint(failedJoint)
               + ", which is outside the allowable range of [" + failedJoint.getJointLowerLimit() + ", " + failedJoint.getJointUpperLimit() + "]";
      }
      return "";
   }
   
   public void cropInitialSimPoints(int numberOfDataPointsToCrop)
   {
      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(numberOfDataPointsToCrop);
      scs.setInPoint();
      scs.cropBuffer();
   }
   
   public void cropFirstPoint()
   {
      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
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
