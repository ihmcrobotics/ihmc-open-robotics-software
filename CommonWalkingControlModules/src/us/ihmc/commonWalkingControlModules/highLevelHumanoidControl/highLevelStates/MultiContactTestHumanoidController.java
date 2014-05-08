package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.trajectories.ChangeableConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantConfigurationProvider;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;

import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;

public class MultiContactTestHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   public final static HighLevelState controllerState = HighLevelState.MULTI_CONTACT;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPT_NULLSPACE;

   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoM", worldFrame, registry);

   private final DesiredFootPoseProvider footPoseProvider;

   private final SideDependentList<ChangeableConfigurationProvider> desiredConfigurationProviders = 
         new SideDependentList<ChangeableConfigurationProvider>();

   private final CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule;
   private final OneDoFJoint[] neckJoints;

   public MultiContactTestHumanoidController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
           CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule,
           MomentumBasedController momentumBasedController,
           WalkingControllerParameters walkingControllerParameters, 
           LidarControllerInterface lidarControllerInterface,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(variousWalkingProviders, variousWalkingManagers, momentumBasedController, walkingControllerParameters,
            lidarControllerInterface, dynamicGraphicObjectsListRegistry, controllerState);

      this.footPoseProvider = variousWalkingProviders.getDesiredFootPoseProvider();
      this.momentumRateOfChangeControlModule = momentumRateOfChangeControlModule;

      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      int spineJacobianId = momentumBasedController.getOrCreateGeometricJacobian(pelvis, chest, chest.getBodyFixedFrame());
      variousWalkingManagers.getChestOrientationManager().setUp(pelvis, spineJacobianId, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0);

      this.neckJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, fullRobotModel.getHead()), OneDoFJoint.class);

      setupFootControlModules();
   }

   protected void setupFootControlModules()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);
         int jacobianId = legJacobianIds.get(robotSide);

         final ConstantConfigurationProvider currentConfigurationProvider = new ConstantConfigurationProvider(new FramePose(foot.getBodyFrame()));
         final ChangeableConfigurationProvider desiredConfigurationProvider =
            new ChangeableConfigurationProvider(footPoseProvider.getDesiredFootPose(robotSide));

         desiredConfigurationProviders.put(robotSide, desiredConfigurationProvider);

         ConstantDoubleProvider footTrajectoryTimeProvider = new ConstantDoubleProvider(1.0);
         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(controlDT, foot, jacobianId, robotSide, /*poseTrajectoryGenerator,*/ null,
                                                                null, null, walkingControllerParameters,
                                                                footTrajectoryTimeProvider, currentConfigurationProvider, null, desiredConfigurationProvider,
                                                                currentConfigurationProvider, null, desiredConfigurationProvider, null,
                                                                null, momentumBasedController, registry);
         endEffectorControlModule.setSwingGains(100.0, 200.0, 200.0, 1.0, 1.0);
         endEffectorControlModule.setHoldGains(100.0, 200.0, 0.1);
         endEffectorControlModule.setToeOffGains(0.0, 200.0, 0.1);

         footEndEffectorControlModules.put(robotSide, endEffectorControlModule);
      }
   }

   public void initialize()
   {
      super.initialize();

      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);
      FramePoint currentCoM = new FramePoint(momentumBasedController.getCenterOfMassFrame());
      currentCoM.changeFrame(desiredCoMPosition.getReferenceFrame());
      desiredCoMPosition.set(currentCoM);

      FrameOrientation currentPelvisOrientaton = new FrameOrientation(referenceFrames.getPelvisFrame());
      currentPelvisOrientaton.changeFrame(desiredPelvisOrientation.getReferenceFrame());
      desiredPelvisOrientation.set(currentPelvisOrientaton);

      // keep desired pelvis orientation as it is
      desiredPelvisAngularVelocity.set(0.0, 0.0, 0.0);
      desiredPelvisAngularAcceleration.set(0.0, 0.0, 0.0);
   }

   protected void doCoMControl()
   {
      momentumRateOfChangeControlModule.getDesiredCoMPositionInputPort().setData(desiredCoMPosition.getFramePointCopy());

      momentumRateOfChangeControlModule.startComputation();
      momentumRateOfChangeControlModule.waitUntilComputationIsDone();
      MomentumRateOfChangeData momentumRateOfChangeData = momentumRateOfChangeControlModule.getMomentumRateOfChangeOutputPort().getData();
      momentumBasedController.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
   }

   protected void doFootControl()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);

         if (footPoseProvider.checkForNewPose(robotSide))
         {
            FramePose newFootPose = footPoseProvider.getDesiredFootPose(robotSide);
            desiredConfigurationProviders.get(foot).set(newFootPose);
            footEndEffectorControlModules.get(robotSide).resetCurrentState();
         }
      }

      super.doFootControl();
   }

   protected void doHeadControl()
   {
      for (OneDoFJoint neckJoint : neckJoints)
      {
         momentumBasedController.doPDControl(neckJoint, 100.0, 20.0, 0.0, 0.0, 10.0, 1000.0);
      }
   }

   public void setFootInContact(RobotSide robotSide, boolean inContact)
   {
      if (feet == null)
         return;

      if (inContact)
      {
         setFlatFootContactState(robotSide);
      }
      else
      {
         setContactStateForSwing(robotSide);
      }
   }

   public void setHandInContact(RobotSide robotSide, boolean inContact)
   {
      ContactablePlaneBody handPalm = handPalms.get(robotSide);

      if (inContact)
      {
         // TODO: If we know the surface normal here, use it.
         FrameVector normalContactVector = null;
         momentumBasedController.setPlaneContactStateFullyConstrained(handPalm, coefficientOfFriction.getDoubleValue(), normalContactVector);
      }
      else
      {
         momentumBasedController.setPlaneContactStateFree(handPalm);
      }
   }

   private void setFlatFootContactState(RobotSide robotSide)
   {
      footEndEffectorControlModules.get(robotSide).setContactState(ConstraintType.FULL);
   }

   private void setContactStateForSwing(RobotSide robotSide)
   {
      // Initialize desired foot pose to the actual, so no surprising behavior
      ReferenceFrame footFrame = footEndEffectorControlModules.get(robotSide).getEndEffectorFrame();
      desiredConfigurationProviders.get(robotSide).set(new FramePose(footFrame));

      footEndEffectorControlModules.get(robotSide).doSingularityEscapeBeforeTransitionToNextState();
      footEndEffectorControlModules.get(robotSide).setContactState(ConstraintType.MOVE_STRAIGHT);
   }
}
