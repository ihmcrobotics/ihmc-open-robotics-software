package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.YoVariableLimitChecker;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class RobotJointLimitWatcher implements RobotController
{
   protected final YoVariableRegistry registry = new YoVariableRegistry("JointLimits");

   protected final DoubleYoVariable[] variablesToTrack;
   protected final YoVariableLimitChecker[] limitCheckers;
   
   protected final OneDoFJoint[] oneDoFJoints;
   private SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   
   protected YoVariableRegistry doNotRegister = new YoVariableRegistry("DoNotRegister");

   public RobotJointLimitWatcher(OneDoFJoint[] oneDoFJoints, SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly)
   {
      this.oneDoFJoints = oneDoFJoints;
      this.sensorRawOutputMapReadOnly = sensorRawOutputMapReadOnly;
      
      int numberOfJoints = oneDoFJoints.length;

      variablesToTrack = new DoubleYoVariable[numberOfJoints];
      limitCheckers = new YoVariableLimitChecker[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];
         variablesToTrack[i] = new DoubleYoVariable(oneDoFJoint.getName(), doNotRegister);

         double thresholdPercentage = 0.02;
         double range = oneDoFJoint.getJointLimitUpper() - oneDoFJoint.getJointLimitLower();
         double thresholdAmount = range * thresholdPercentage;
         double lowerLimit = oneDoFJoint.getJointLimitLower() + thresholdAmount;
         double upperLimit = oneDoFJoint.getJointLimitUpper() - thresholdAmount;

         limitCheckers[i] = new YoVariableLimitChecker(variablesToTrack[i], "limit", lowerLimit, upperLimit, registry);
      }
   }
   
   public RobotJointLimitWatcher(OneDoFJoint[] oneDoFJoints)
   {
      this(oneDoFJoints, null);
   }

   public void doControl()
   {
      for (int i = 0; i < limitCheckers.length; i++)
      {
         // If created with sensorRawOutputMapReadOnly use those rather than the oneDoFJoint values.
         // Otherwise, various filtering and elasticity compensation might get in the way
         // and you get a wrong output.
         if (sensorRawOutputMapReadOnly != null)
         {
            double jointPositionRawOutput = sensorRawOutputMapReadOnly.getJointPositionRawOutput(oneDoFJoints[i]);
            variablesToTrack[i].set(jointPositionRawOutput);
            limitCheckers[i].update();
         }
         else
         {
            variablesToTrack[i].set(oneDoFJoints[i].getQ());
            limitCheckers[i].update();
         }
      }
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return null;    // To change body of implemented methods use File | Settings | File Templates.
   }

   public String getDescription()
   {
      return null;    // To change body of implemented methods use File | Settings | File Templates.
   }
}
