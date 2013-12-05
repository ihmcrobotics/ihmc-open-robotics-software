package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHeadOrientationProvider;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;

public class HeadOrientationManager
{   
   private final HeadOrientationControlModule headOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;
   private int jacobianId = -1;

   public HeadOrientationManager(MomentumBasedController momentumBasedController, HeadOrientationControlModule headOrientationControlModule,
                                 DesiredHeadOrientationProvider desiredHeadOrientationProvider)
   {
      this.momentumBasedController = momentumBasedController;
      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      
      this.headOrientationControlModule = headOrientationControlModule;
   }

   public void compute()
   {
      if (desiredHeadOrientationProvider != null)
      {
         if (desiredHeadOrientationProvider.isNewHeadOrientationInformationAvailable())
         {
            headOrientationControlModule.setOrientationToTrack(desiredHeadOrientationProvider.getDesiredHeadOrientation());
         }
         if (desiredHeadOrientationProvider.isNewLookAtInformationAvailable())
         {
            headOrientationControlModule.setPointToTrack(desiredHeadOrientationProvider.getLookAtPoint());
         }
      }

      if (jacobianId >= 0)
      {
         headOrientationControlModule.compute();

         TaskspaceConstraintData taskspaceConstraintData = headOrientationControlModule.getTaskspaceConstraintData();
         momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
      }
   }
   
   public int createJacobian(FullRobotModel fullRobotModel, String[] headOrientationControlJointNames)
   {
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] headOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, headOrientationControlJointNames);

      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(headOrientationControlJoints, headOrientationControlModule.getHead().getBodyFixedFrame());
      return jacobianId;
   }
   
   public void setUp(RigidBody base, int jacobianId)
   {
      this.jacobianId = jacobianId;
      headOrientationControlModule.setBase(base);
      headOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
   }
   
   public void setUp(RigidBody base, int jacobianId, double proportionalGainX, double proportionalGainY, double proportionalGainZ,
                     double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      this.jacobianId = jacobianId;
      headOrientationControlModule.setBase(base);
      headOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
      headOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
      headOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setControlGains(double proportionalGain, double derivativeGain)
   {
      headOrientationControlModule.setProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      headOrientationControlModule.setDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
   }
   
   public void turnOff()
   {
      setUp(null, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   
}
