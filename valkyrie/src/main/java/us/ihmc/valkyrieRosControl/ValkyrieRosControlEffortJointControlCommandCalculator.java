package us.ihmc.valkyrieRosControl;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieRosControlEffortJointControlCommandCalculator
{
   private final YoEffortJointHandleHolder yoEffortJointHandleHolder;

   private final PIDController pidController;
   private final YoDouble tauOffset;

   private final double controlDT;

   public ValkyrieRosControlEffortJointControlCommandCalculator(YoEffortJointHandleHolder yoEffortJointHandleHolder, double torqueOffset, double controlDT,
                                                                YoRegistry parentRegistry)
   {
      this.yoEffortJointHandleHolder = yoEffortJointHandleHolder;

      this.controlDT = controlDT;

      String pdControllerBaseName = yoEffortJointHandleHolder.getName();
      YoRegistry registry = new YoRegistry(pdControllerBaseName + "Command");

      pidController = new PIDController(pdControllerBaseName + "LowLevelControl", registry);
      tauOffset = new YoDouble("tau_offset_" + pdControllerBaseName, registry);

      pidController.setMaxIntegralError(50.0);
      pidController.setCumulativeError(0.0);

      tauOffset.set(torqueOffset);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      pidController.setCumulativeError(0.0);
   }

   public void computeAndUpdateJointTorque()
   {
      JointDesiredOutputReadOnly desiredOutput = yoEffortJointHandleHolder.getDesiredJointData();
      pidController.setProportionalGain(desiredOutput.hasStiffness() ? desiredOutput.getStiffness() : 0.0);
      pidController.setDerivativeGain(desiredOutput.hasDamping() ? desiredOutput.getDamping() : 0.0);

      OneDoFJointBasics oneDoFJoint = yoEffortJointHandleHolder.getOneDoFJoint();
      
      double q, qd;

      if (oneDoFJoint != null)
      {
         q = oneDoFJoint.getQ();
         qd = oneDoFJoint.getQd();
      }
      else
      {
         q = yoEffortJointHandleHolder.getQ();
         qd = yoEffortJointHandleHolder.getQd();
      }

      double qDesired = desiredOutput.hasDesiredPosition() ? desiredOutput.getDesiredPosition() : q;
      double qdDesired = desiredOutput.hasDesiredVelocity() ? desiredOutput.getDesiredVelocity() : qd;

      double fb_tau = pidController.compute(q, qDesired, qd, qdDesired, controlDT);
      double ff_tau = desiredOutput.hasDesiredTorque() ? desiredOutput.getDesiredTorque() : 0.0;

      double desiredEffort = fb_tau + ff_tau + tauOffset.getDoubleValue();
      desiredEffort = EuclidCoreTools.clamp(desiredEffort, yoEffortJointHandleHolder.getTauMax());

      yoEffortJointHandleHolder.setDesiredEffort(desiredEffort);
   }

   public void subtractTorqueOffset(double torqueOffset)
   {
      tauOffset.sub(torqueOffset);
   }
}
