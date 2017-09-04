package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JointControlCalculator
{
   private final PIDController pidPositionController;
   private final DeltaLimitedYoVariable positionStepSizeLimiter;
   private final OneDoFJoint oneDoFJoint;
   private final double controlDT;

   public JointControlCalculator(String nameSuffix, OneDoFJoint oneDoFJoint, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;
      this.controlDT = controlDT;
      String namePrefix = oneDoFJoint.getName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + nameSuffix + "JointControlCalculator");

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
      positionStepSizeLimiter.updateOutput(oneDoFJoint.getqDesired(), oneDoFJoint.getqDesired());
   }

   public void computeAndUpdateJointControl(LowLevelJointData positionControllerDesiredsToPack, double masterPositionGain)
   {
      double currentJointAngle = oneDoFJoint.getQ();
      double desiredPosition = masterPositionGain * positionControllerDesiredsToPack.getDesiredPosition();

      //positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);
      //desiredPosition = positionStepSizeLimiter.getDoubleValue();

      double qd = oneDoFJoint.getQd();
      double qDesired = positionControllerDesiredsToPack.getDesiredPosition();
      double qdDesired = positionControllerDesiredsToPack.getDesiredVelocity();
      double desiredTorque = masterPositionGain * pidPositionController.compute(currentJointAngle, qDesired, qd, qdDesired, controlDT);

      positionControllerDesiredsToPack.setDesiredPosition(desiredPosition);
      positionControllerDesiredsToPack.setDesiredTorque(desiredTorque);
   }
}
