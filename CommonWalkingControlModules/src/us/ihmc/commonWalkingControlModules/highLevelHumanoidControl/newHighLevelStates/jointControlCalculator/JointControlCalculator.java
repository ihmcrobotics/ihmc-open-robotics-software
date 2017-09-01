package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointControlCalculator
{
   private final PIDController pidPositionController;
   private final DeltaLimitedYoVariable positionStepSizeLimiter;
   private final YoDouble initialAngle;

   private final OneDoFJoint oneDoFJoint;
   private final double controlDT;

   public JointControlCalculator(String nameSuffix, OneDoFJoint oneDoFJoint, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;
      this.controlDT = controlDT;
      String namePrefix = oneDoFJoint.getName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + nameSuffix + "JointControlCalculator");

      this.initialAngle = new YoDouble(namePrefix + nameSuffix + "InitialAngle", registry);

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(namePrefix + nameSuffix + "PositionStepSizeLimiter", registry, 0.15);

      pidPositionController = new PIDController(namePrefix + nameSuffix, registry);

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
      positionStepSizeLimiter.updateOutput(oneDoFJoint.getQ(), oneDoFJoint.getQ());

      double q = oneDoFJoint.getQ();
      double jointLimitLower = oneDoFJoint.getJointLimitLower();
      double jointLimitUpper = oneDoFJoint.getJointLimitUpper();
      q = MathTools.clamp(q, jointLimitLower, jointLimitUpper);

      initialAngle.set(q);
   }

   public void computeAndUpdateJointControl(LowLevelJointData positionControllerDesireds, double masterPositionGain)
   {
      double currentJointAngle = oneDoFJoint.getQ();
      double desiredPosition = masterPositionGain * positionControllerDesireds.getDesiredPosition();

      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);
      desiredPosition = positionStepSizeLimiter.getDoubleValue();

      double qd = oneDoFJoint.getQd();
      double qDesired = positionControllerDesireds.getDesiredPosition();
      double qdDesired = positionControllerDesireds.getDesiredVelocity();
      double desiredTorque = masterPositionGain * pidPositionController.compute(currentJointAngle, qDesired, qd, qdDesired, controlDT);

      positionControllerDesireds.setDesiredPosition(desiredPosition);
      positionControllerDesireds.setDesiredTorque(desiredTorque);
   }
}
