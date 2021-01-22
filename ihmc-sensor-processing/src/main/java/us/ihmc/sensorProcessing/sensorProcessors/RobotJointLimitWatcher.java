package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.YoVariableLimitChecker;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RobotJointLimitWatcher implements RobotController
{
   protected final YoRegistry registry = new YoRegistry("JointLimits");

   protected final YoDouble[] variablesToTrack;
   protected final YoVariableLimitChecker[] limitCheckers;
   
   protected final OneDoFJointBasics[] oneDoFJoints;
   private SensorOutputMapReadOnly rawSensorOutputMap;
   
   protected YoRegistry doNotRegister = new YoRegistry("DoNotRegister");

   public RobotJointLimitWatcher(OneDoFJointBasics[] oneDoFJoints, SensorOutputMapReadOnly rawSensorOutputMap)
   {
      this.oneDoFJoints = oneDoFJoints;
      this.rawSensorOutputMap = rawSensorOutputMap;
      
      int numberOfJoints = oneDoFJoints.length;

      variablesToTrack = new YoDouble[numberOfJoints];
      limitCheckers = new YoVariableLimitChecker[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJointBasics oneDoFJoint = oneDoFJoints[i];
         variablesToTrack[i] = new YoDouble(oneDoFJoint.getName(), doNotRegister);

         double thresholdPercentage = 0.02;
         double range = oneDoFJoint.getJointLimitUpper() - oneDoFJoint.getJointLimitLower();
         double thresholdAmount = range * thresholdPercentage;
         double lowerLimit = oneDoFJoint.getJointLimitLower() + thresholdAmount;
         double upperLimit = oneDoFJoint.getJointLimitUpper() - thresholdAmount;

         limitCheckers[i] = new YoVariableLimitChecker(variablesToTrack[i], "limit", lowerLimit, upperLimit, registry);
      }
   }
   
   public RobotJointLimitWatcher(OneDoFJointBasics[] oneDoFJoints)
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
         if (rawSensorOutputMap != null)
         {
            OneDoFJointStateReadOnly jointOutput = rawSensorOutputMap.getOneDoFJointOutput(oneDoFJoints[i]);
            if (jointOutput != null)
            {
               double jointPositionRawOutput = jointOutput.getPosition();
               variablesToTrack[i].set(jointPositionRawOutput);
               limitCheckers[i].update();
            }
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

   public YoRegistry getYoRegistry()
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
