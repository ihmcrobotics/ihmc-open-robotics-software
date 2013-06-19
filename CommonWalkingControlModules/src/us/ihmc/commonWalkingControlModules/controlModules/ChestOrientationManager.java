package us.ihmc.commonWalkingControlModules.controlModules;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;


public class ChestOrientationManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ChestOrientationControlModule chestOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private final DoubleYoVariable timeLastUpdated = new DoubleYoVariable("chestDesiredTimelastUpdated", registry);

   public ChestOrientationManager(MomentumBasedController momentumBasedController, ChestOrientationControlModule chestOrientationControlModule, YoVariableRegistry parentRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      this.chestOrientationControlModule = chestOrientationControlModule;
      timeLastUpdated.set(Double.NaN);
      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      GeometricJacobian jacobian = chestOrientationControlModule.getJacobian();

      if (jacobian != null)
      {
         chestOrientationControlModule.compute();

         momentumBasedController.setDesiredSpatialAcceleration(jacobian, chestOrientationControlModule.getTaskspaceConstraintData());
      }
   }

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector desiredAngularAcceleration)
   {
      chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      timeLastUpdated.set(momentumBasedController.getYoTime().getDoubleValue());
   }

   public GeometricJacobian createJacobian(FullRobotModel fullRobotModel, String[] chestOrientationControlJointNames)
   {
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      return createJacobian(chestOrientationControlJoints);
   }

   public GeometricJacobian createJacobian(InverseDynamicsJoint[] chestOrientationControlJoints)
   {
      GeometricJacobian spineJacobian = new GeometricJacobian(chestOrientationControlJoints, chestOrientationControlModule.getChest().getBodyFixedFrame());

      return spineJacobian;
   }

   public void setUp(RigidBody base, GeometricJacobian spineJacobian, double proportionalGainX, double proportionalGainY, double proportionalGainZ,
                     double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      chestOrientationControlModule.setBase(base);
      chestOrientationControlModule.setJacobian(spineJacobian);
      chestOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
      chestOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void turnOff()
   {
      setUp(null, null, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   public boolean areDesiredsValid()
   {
      double timeSinceLastUpdate = momentumBasedController.getYoTime().getDoubleValue() - timeLastUpdated.getDoubleValue();
      return timeSinceLastUpdate < 1.5 * momentumBasedController.getControlDT();
   }

   public FrameOrientation getDesiredChestOrientation()
   {
      return chestOrientationControlModule.getDesiredFrameOrientation();
   }
}
