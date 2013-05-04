package us.ihmc.darpaRoboticsChallenge.sensors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableLimitChecker;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/4/13
 * Time: 3:09 PM
 * To change this template use File | Settings | File Templates.
 */
public class RobotJointLimitWatcher implements RobotController
{
   private final YoVariableRegistry registry;

   private DoubleYoVariable[] variablesToTrack;

   private YoVariableLimitChecker[] limitCheckers;
   private OneDoFJoint[] oneDoFJoints;
   public RobotJointLimitWatcher(OneDoFJoint[] oneDoFJoints)
   {
      registry = new YoVariableRegistry("JointLimits");
      this.oneDoFJoints = oneDoFJoints;

      int numberOfJoints = oneDoFJoints.length;

      variablesToTrack = new DoubleYoVariable[numberOfJoints];
      limitCheckers = new YoVariableLimitChecker[numberOfJoints];

      YoVariableRegistry doNotRegister = new YoVariableRegistry("DoNotRegister");
      for (int i=0; i<numberOfJoints; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];
         variablesToTrack[i] = new DoubleYoVariable(oneDoFJoint.getName(), doNotRegister);

         double reductionFactor = 0.95;
         double lowerLimit = reductionFactor * oneDoFJoint.getJointLimitLower();
         double upperLimit = reductionFactor * oneDoFJoint.getJointLimitUpper();

         limitCheckers[i] = new YoVariableLimitChecker(variablesToTrack[i], lowerLimit, upperLimit, registry);
      }
   }

   public void doControl()
   {
      for(int i=0; i<limitCheckers.length; i++)
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
      return null;  //To change body of implemented methods use File | Settings | File Templates.
   }

   public String getDescription()
   {
      return null;  //To change body of implemented methods use File | Settings | File Templates.
   }
}
