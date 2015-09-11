package us.ihmc.darpaRoboticsChallenge.sensors;

import java.util.ArrayList;

import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.robotics.math.YoVariableLimitChecker;
import us.ihmc.robotics.screwTheory.OneDoFJoint;


public class RobotJointLimitWatcherForUI extends RobotJointLimitWatcher
{
   private YoVariableLimitChecker[] closeToLimitCheckers;

   public RobotJointLimitWatcherForUI(OneDoFJoint[] oneDoFJoints)
   {
      super(oneDoFJoints);

      int numberOfJoints = oneDoFJoints.length;
      closeToLimitCheckers = new YoVariableLimitChecker[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];

         double thresholdPercentage = 0.02;
         double range = oneDoFJoint.getJointLimitUpper() - oneDoFJoint.getJointLimitLower();
         double thresholdAmount = range * thresholdPercentage;
         double lowerLimit = oneDoFJoint.getJointLimitLower() + thresholdAmount + (range * 0.1);
         double upperLimit = oneDoFJoint.getJointLimitUpper() - thresholdAmount - (range * 0.1);

         closeToLimitCheckers[i] = new YoVariableLimitChecker(variablesToTrack[i], "approaching", lowerLimit, upperLimit, registry);
      }
   }

   public void doControl()
   {
      for (int i = 0; i < limitCheckers.length; i++)
      {
         variablesToTrack[i].set(oneDoFJoints[i].getQ());
         limitCheckers[i].update();
         closeToLimitCheckers[i].update();
      }
   }

   public ArrayList<String> getJointsAtLimits()
   {
      ArrayList<String> jointsAtLimits = new ArrayList<String>();

      for (int i = 0; i < limitCheckers.length; i++)
      {
         String jointName = oneDoFJoints[i].getName();
         YoVariableLimitChecker.Status jointStatus = limitCheckers[i].getStatus();
         if (!jointStatus.equals(YoVariableLimitChecker.Status.IN_RANGE))
         {
            jointsAtLimits.add(jointName);
         }
      }

      return jointsAtLimits;
   }

   public ArrayList<String> getJointsApproachingLimits()
   {
      ArrayList<String> jointsApproachingLimits = new ArrayList<String>();

      for (int i = 0; i < closeToLimitCheckers.length; i++)
      {
         String jointName = oneDoFJoints[i].getName();
         YoVariableLimitChecker.Status jointStatus = closeToLimitCheckers[i].getStatus();
         if (!jointStatus.equals(YoVariableLimitChecker.Status.IN_RANGE))
         {
            jointsApproachingLimits.add(jointName);
         }
      }

      return jointsApproachingLimits;
   }
}
