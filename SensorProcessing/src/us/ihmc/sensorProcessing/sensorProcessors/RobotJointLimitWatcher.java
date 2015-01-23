package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.YoVariableLimitChecker;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/4/13
 * Time: 3:09 PM
 * To change this template use File | Settings | File Templates.
 */
public class RobotJointLimitWatcher implements RobotController
{
   protected final YoVariableRegistry registry;

   protected DoubleYoVariable[] variablesToTrack;

   protected YoVariableLimitChecker[] limitCheckers;
   protected OneDoFJoint[] oneDoFJoints;


   protected YoVariableRegistry doNotRegister = new YoVariableRegistry("DoNotRegister");

   public RobotJointLimitWatcher(OneDoFJoint[] oneDoFJoints)
   {
      registry = new YoVariableRegistry("JointLimits");
      this.oneDoFJoints = oneDoFJoints;

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

   public void doControl()
   {
      for (int i = 0; i < limitCheckers.length; i++)
      {
         variablesToTrack[i].set(oneDoFJoints[i].getQ());
         limitCheckers[i].update();
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
