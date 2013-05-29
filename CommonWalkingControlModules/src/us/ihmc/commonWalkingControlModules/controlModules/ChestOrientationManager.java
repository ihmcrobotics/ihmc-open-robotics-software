package us.ihmc.commonWalkingControlModules.controlModules;

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
   private final ChestOrientationControlModule chestOrientationControlModule;
   private final MomentumBasedController momentumBasedController;

   public ChestOrientationManager(MomentumBasedController momentumBasedController, ChestOrientationControlModule chestOrientationControlModule)
   {
      this.momentumBasedController = momentumBasedController;
      this.chestOrientationControlModule = chestOrientationControlModule;
   }

   public void compute()
   {
      GeometricJacobian jacobian = chestOrientationControlModule.getJacobian();

      if (jacobian != null)
      {
         chestOrientationControlModule.compute();

         momentumBasedController.setDesiredSpatialAcceleration(jacobian,
               chestOrientationControlModule.getTaskspaceConstraintData());
      }
   }

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector desiredAngularAcceleration)
   {
      chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }

   public GeometricJacobian createJacobian(FullRobotModel fullRobotModel, RigidBody base, String[] chestOrientationControlJointNames)
   {
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);
      
      return createJacobian(base, chestOrientationControlJoints);
   }
   
   public GeometricJacobian createJacobian(RigidBody base, InverseDynamicsJoint[] chestOrientationControlJoints)
   {
      GeometricJacobian spineJacobian = new GeometricJacobian(chestOrientationControlJoints, chestOrientationControlModule.getChest().getBodyFixedFrame());
      return spineJacobian;
   }

   public void setBaseAndJacobian(RigidBody base, GeometricJacobian spineJacobian)
   {
      chestOrientationControlModule.setBase(base);
      chestOrientationControlModule.setJacobian(spineJacobian);
   }

   public void turnOff()
   {
      setBaseAndJacobian(null, null);
   }

}
