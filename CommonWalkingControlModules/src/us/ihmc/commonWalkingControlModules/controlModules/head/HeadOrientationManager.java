package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class HeadOrientationManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final HeadOrientationControlModule headOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;


   public HeadOrientationManager(MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters, DesiredHeadOrientationProvider desiredHeadOrientationProvider, YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      
      this.headOrientationControlModule = setupHeadOrientationControlModule(walkingControllerParameters, registry, dynamicGraphicObjectsListRegistry);
   }

   public void compute()
   { 
      if (headOrientationControlModule != null)
      {
         headOrientationControlModule.compute();
         GeometricJacobian jacobian = headOrientationControlModule.getJacobian();
         TaskspaceConstraintData taskspaceConstraintData = headOrientationControlModule.getTaskspaceConstraintData();
         momentumBasedController.setDesiredSpatialAcceleration(jacobian,
               taskspaceConstraintData);
      }
   }
   
   private HeadOrientationControlModule setupHeadOrientationControlModule(WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      parentRegistry.addChild(registry);
      
      String[] defaultHeadOrientationControlJointNames = walkingControllerParameters.getDefaultHeadOrientationControlJointNames();
      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      CommonWalkingReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] headOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, defaultHeadOrientationControlJointNames);

      if (headOrientationControlJoints.length <= 0)
         return null;

      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody head = fullRobotModel.getHead();
      RigidBody elevator = fullRobotModel.getElevator();
      GeometricJacobian neckJacobian = new GeometricJacobian(headOrientationControlJoints, head.getBodyFixedFrame());
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();

      HeadOrientationControlModule headOrientationControlModule = new HeadOrientationControlModule(neckJacobian, pelvis, elevator, twistCalculator,
            pelvisZUpFrame, chestFrame, walkingControllerParameters, registry,
                                                                     dynamicGraphicObjectsListRegistry);

      // Setting initial head pitch
      // This magic number (0.67) is a good default head pitch for getting good LIDAR point coverage of ground by feet
      // it would be in DRC config parameters, but the would require updating several nested constructors with an additional parameter
      FrameOrientation orientation = new FrameOrientation(pelvisZUpFrame, 0.0, 0.67, 0.0);
      headOrientationControlModule.setOrientationToTrack(new FrameOrientation(orientation));
      double headKp = 40.0;
      double headZeta = 1.0;
      double headKd = GainCalculator.computeDerivativeGain(headKp, headZeta);
      headOrientationControlModule.setProportionalGains(headKp, headKp, headKp);
      headOrientationControlModule.setDerivativeGains(headKd, headKd, headKd);

      if (desiredHeadOrientationProvider != null)
         desiredHeadOrientationProvider.setHeadOrientationControlModule(headOrientationControlModule);

      return headOrientationControlModule;
   }

}
