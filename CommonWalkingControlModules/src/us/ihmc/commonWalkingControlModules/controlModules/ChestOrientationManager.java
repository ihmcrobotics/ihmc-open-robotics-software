package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.DoubleYoVariable;


public class ChestOrientationManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ChestOrientationControlModule chestOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private final DoubleYoVariable timeLastUpdated = new DoubleYoVariable("chestDesiredTimelastUpdated", registry);
   private int jacobianId = -1;

   public ChestOrientationManager(MomentumBasedController momentumBasedController, ChestOrientationControlModule chestOrientationControlModule, YoVariableRegistry parentRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      this.chestOrientationControlModule = chestOrientationControlModule;
      timeLastUpdated.set(Double.NaN);
      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      if (jacobianId >= 0)
      {
         chestOrientationControlModule.compute();

         momentumBasedController.setDesiredSpatialAcceleration(jacobianId, chestOrientationControlModule.getTaskspaceConstraintData());
      }
   }

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector desiredAngularAcceleration)
   {
      chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      timeLastUpdated.set(momentumBasedController.getYoTime().getDoubleValue());
   }

   public int createJacobian(FullRobotModel fullRobotModel, String[] chestOrientationControlJointNames)
   {
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chestOrientationControlJoints, chestOrientationControlModule.getChest().getBodyFixedFrame());
      return jacobianId;
   }

   public void setUp(RigidBody base, int jacobianId)
   {
      this.jacobianId = jacobianId;
      chestOrientationControlModule.setBase(base);
      chestOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
   }
   
   public void setUp(RigidBody base, int jacobianId, double proportionalGainX, double proportionalGainY, double proportionalGainZ,
                     double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      this.jacobianId = jacobianId;
      chestOrientationControlModule.setBase(base);
      chestOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
      chestOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
      chestOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }
   
   public void setControlGains(double proportionalGain, double derivativeGain)
   {
      chestOrientationControlModule.setProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      chestOrientationControlModule.setDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
   }

   public void turnOff()
   {
      setUp(null, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   public boolean areDesiredsValid()
   {
      double timeSinceLastUpdate = momentumBasedController.getYoTime().getDoubleValue() - timeLastUpdated.getDoubleValue();
      return timeSinceLastUpdate < 1.5 * momentumBasedController.getControlDT();
   }
}
