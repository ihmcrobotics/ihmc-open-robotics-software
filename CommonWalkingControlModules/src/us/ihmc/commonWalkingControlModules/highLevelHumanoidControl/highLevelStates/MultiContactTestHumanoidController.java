package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import org.apache.commons.lang.mutable.MutableBoolean;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packets.HandLoadBearingPacket;
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

import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;

public class MultiContactTestHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   public final static HighLevelState controllerState = HighLevelState.MULTI_CONTACT;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPT_NULLSPACE;

   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoM", worldFrame, registry);
   private final YoFrameOrientation desiredPelvisYawPitchRoll = new YoFrameOrientation("desiredPelvis", worldFrame, registry);
   private final YoFrameOrientation desiredChestYawPitchRoll = new YoFrameOrientation("desiredChest", worldFrame, registry);

   private final DesiredFootPoseProvider footPoseProvider;

   private final SideDependentList<ChangeableConfigurationProvider> desiredConfigurationProviders = 
         new SideDependentList<ChangeableConfigurationProvider>();

   private final CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule;
   private final OneDoFJoint[] neckJoints;
   
   private final SideDependentList<MutableBoolean> initializeHandsInContact = new SideDependentList<>(new MutableBoolean(), new MutableBoolean());
   private final SideDependentList<MutableBoolean> initializeFeetInContact = new SideDependentList<>(new MutableBoolean(), new MutableBoolean());

   public MultiContactTestHumanoidController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule, MomentumBasedController momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, LidarControllerInterface lidarControllerInterface,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(variousWalkingProviders, variousWalkingManagers, momentumBasedController, walkingControllerParameters, controllerState);

      this.footPoseProvider = variousWalkingProviders.getDesiredFootPoseProvider();
      this.momentumRateOfChangeControlModule = momentumRateOfChangeControlModule;

      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      int spineJacobianId = momentumBasedController.getOrCreateGeometricJacobian(pelvis, chest, chest.getBodyFixedFrame());
      chestOrientationManager.setUp(pelvis, spineJacobianId, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0);

      this.neckJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, fullRobotModel.getHead()), OneDoFJoint.class);

      desiredPelvisYawPitchRoll.attachVariableChangedListener(new VariableChangedListener()
      {
         private final FrameOrientation tempOrientation = new FrameOrientation(desiredPelvisYawPitchRoll.getReferenceFrame());
         
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            desiredPelvisYawPitchRoll.getFrameOrientation(tempOrientation);
            desiredPelvisOrientation.set(tempOrientation);
         }
      });

      desiredChestYawPitchRoll.attachVariableChangedListener(new VariableChangedListener()
      {
         private final FrameOrientation tempOrientation = new FrameOrientation(desiredChestYawPitchRoll.getReferenceFrame());
         private final FrameVector zeroFrameVector = new FrameVector(desiredChestYawPitchRoll.getReferenceFrame());
         
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            desiredChestYawPitchRoll.getFrameOrientation(tempOrientation);
            chestOrientationManager.setDesireds(tempOrientation, zeroFrameVector, zeroFrameVector);
         }
      });
      
      setupFootControlModules();
   }

   public void initializeContactStates(SideDependentList<Boolean> areHandsInContact, SideDependentList<Boolean> areFeetInContact)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         initializeHandsInContact.get(robotSide).setValue(areHandsInContact.get(robotSide));
         initializeFeetInContact.get(robotSide).setValue(areFeetInContact.get(robotSide));
      }
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
         FootControlModule footControlModule = new FootControlModule(controlDT, foot, jacobianId, robotSide, null,
               null, null, walkingControllerParameters, footTrajectoryTimeProvider, currentConfigurationProvider, 
               /*null,*/ desiredConfigurationProvider, currentConfigurationProvider, /*null,*/ desiredConfigurationProvider, null,
               null, momentumBasedController, registry);
         
         footControlModule.setSwingGains(100.0, 200.0, 200.0, 1.0, 1.0);
         footControlModule.setHoldGains(100.0, 200.0, 0.1);
         footControlModule.setToeOffGains(0.0, 200.0, 0.1);

         footControlModules.put(robotSide, footControlModule);
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
      
      for (RobotSide robotSide : RobotSide.values)
      {
         if (initializeHandsInContact.get(robotSide).booleanValue())
         {
            HandLoadBearingPacket handLoadBearingPacket = new HandLoadBearingPacket(robotSide, true);
            variousWalkingProviders.getDesiredHandLoadBearingProvider().consumeObject(handLoadBearingPacket);
         }
         
         setFootInContact(robotSide, initializeFeetInContact.get(robotSide).booleanValue());
      }
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
            footControlModules.get(robotSide).resetCurrentState();
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

   private void setFootInContact(RobotSide robotSide, boolean inContact)
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

   private void setFlatFootContactState(RobotSide robotSide)
   {
      footControlModules.get(robotSide).setContactState(ConstraintType.FULL);
   }

   private void setContactStateForSwing(RobotSide robotSide)
   {
      // Initialize desired foot pose to the actual, so no surprising behavior
      ReferenceFrame footFrame = footControlModules.get(robotSide).getFootFrame();
      desiredConfigurationProviders.get(robotSide).set(new FramePose(footFrame));

      footControlModules.get(robotSide).doSingularityEscapeBeforeTransitionToNextState();
      footControlModules.get(robotSide).setContactState(ConstraintType.MOVE_STRAIGHT);
   }
}
