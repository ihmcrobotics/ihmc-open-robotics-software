package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHeadOrientationProvider;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class HeadOrientationManager
{   
   private final HeadOrientationControlModule headOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
//   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;


   public HeadOrientationManager(MomentumBasedController momentumBasedController, HeadOrientationControlModule headOrientationControlModule, 
        DesiredHeadOrientationProvider desiredHeadOrientationProvider, YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.momentumBasedController = momentumBasedController;
//      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      
      this.headOrientationControlModule = headOrientationControlModule;
   }

   public void compute()
   { 
      GeometricJacobian jacobian = headOrientationControlModule.getJacobian();
      
      if (jacobian != null)
      {
         headOrientationControlModule.compute();

         TaskspaceConstraintData taskspaceConstraintData = headOrientationControlModule.getTaskspaceConstraintData();
         momentumBasedController.setDesiredSpatialAcceleration(jacobian,
               taskspaceConstraintData);
      }
   }
   
   public GeometricJacobian createJacobian(FullRobotModel fullRobotModel, RigidBody base, String[] headOrientationControlJointNames)
   {
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] headOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, headOrientationControlJointNames);
      
      return createJacobian(base, headOrientationControlJoints);
   }
   
   public GeometricJacobian createJacobian(RigidBody base, InverseDynamicsJoint[] headOrientationControlJoints)
   {
      GeometricJacobian spineJacobian = new GeometricJacobian(headOrientationControlJoints, headOrientationControlModule.getHead().getBodyFixedFrame());
      return spineJacobian;
   }

   public void setBaseAndJacobian(RigidBody base, GeometricJacobian spineJacobian)
   {
      headOrientationControlModule.setBase(base);
      headOrientationControlModule.setJacobian(spineJacobian);
   }

   public void turnOff()
   {
      setBaseAndJacobian(null, null);
   }

}
