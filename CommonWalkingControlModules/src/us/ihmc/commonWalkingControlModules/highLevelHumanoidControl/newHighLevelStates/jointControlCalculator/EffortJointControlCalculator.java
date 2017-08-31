package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;

public class EffortJointControlCalculator
{
   private final PIDController pidPositionController;
   private final YoDouble initialAngle;

   private final OneDoFJoint oneDoFJoint;
   private final double controlDT;

   public EffortJointControlCalculator(String standSuffix, OneDoFJoint oneDoFJoint, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.oneDoFJoint = oneDoFJoint;

      String namePrefix = oneDoFJoint.getName();

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + standSuffix + "Command");

      this.initialAngle = new YoDouble(namePrefix + standSuffix + "InitialAngle", registry);

      pidPositionController = new PIDController(namePrefix + standSuffix, registry);

      pidPositionController.setMaxIntegralError(50.0);
      pidPositionController.setCumulativeError(0.0);

      parentRegistry.addChild(registry);
   }

   public void setProportionalGain(double kp)
   {
      pidPositionController.setProportionalGain(kp);
   }

   public void setDerivativeGain(double kd)
   {
      pidPositionController.setDerivativeGain(kd);
   }

   public void setIntegralGain(double ki)
   {
      pidPositionController.setIntegralGain(ki);
   }

   public void initialize()
   {
      pidPositionController.setCumulativeError(0.0);

      double q = oneDoFJoint.getQ();
      double jointLimitLower = oneDoFJoint.getJointLimitLower();
      double jointLimitUpper = oneDoFJoint.getJointLimitUpper();
      q = MathTools.clamp(q, jointLimitLower, jointLimitUpper);

      initialAngle.set(q);
   }

   public double computeAndUpdateJointTorque(LowLevelJointData positionControllerDesireds, double masterPositionGain)
   {
      double q = oneDoFJoint.getQ();
      double qd = oneDoFJoint.getQd();
      double qDesired = positionControllerDesireds.getDesiredPosition();
      double qdDesired = positionControllerDesireds.getDesiredVelocity();
      double desiredTorque = masterPositionGain * pidPositionController.compute(q, qDesired, qd, qdDesired, controlDT);

      positionControllerDesireds.setDesiredTorque(desiredTorque);
      return desiredTorque;
   }

}
