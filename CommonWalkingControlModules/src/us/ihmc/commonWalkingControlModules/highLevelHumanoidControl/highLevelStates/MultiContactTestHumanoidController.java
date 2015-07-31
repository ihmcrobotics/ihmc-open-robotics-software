package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootPoseProvider;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.packets.manipulation.HandLoadBearingPacket;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.trajectories.providers.ChangeableConfigurationProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class MultiContactTestHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   public final static HighLevelState controllerState = HighLevelState.MULTI_CONTACT;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPT_NULLSPACE;

   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoM", worldFrame, registry);
   private final YoFrameOrientation desiredChestYawPitchRoll = new YoFrameOrientation("desiredChest", worldFrame, registry);

   private final FootPoseProvider footPoseProvider;

   private final SideDependentList<ChangeableConfigurationProvider> desiredConfigurationProviders = 
         new SideDependentList<ChangeableConfigurationProvider>();

   private final CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule;
   private final OneDoFJoint[] neckJoints;
   
   private final SideDependentList<MutableBoolean> initializeHandsInContact = new SideDependentList<>(new MutableBoolean(), new MutableBoolean());
   private final SideDependentList<MutableBoolean> initializeFeetInContact = new SideDependentList<>(new MutableBoolean(), new MutableBoolean());

   private final MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(momentumBasedController.getCenterOfMassFrame());
   
   public MultiContactTestHumanoidController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule, MomentumBasedController momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(variousWalkingProviders, variousWalkingManagers, momentumBasedController, walkingControllerParameters, controllerState);

      this.footPoseProvider = variousWalkingProviders.getDesiredFootPoseProvider();
      this.momentumRateOfChangeControlModule = momentumRateOfChangeControlModule;

      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      int spineJacobianId = momentumBasedController.getOrCreateGeometricJacobian(pelvis, chest, chest.getBodyFixedFrame());
      chestOrientationManager.setUp(pelvis, spineJacobianId, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0);

      this.neckJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, fullRobotModel.getHead()), OneDoFJoint.class);

      desiredChestYawPitchRoll.attachVariableChangedListener(new VariableChangedListener()
      {
         private final FrameOrientation tempOrientation = new FrameOrientation(desiredChestYawPitchRoll.getReferenceFrame());
         private final FrameVector zeroFrameVector = new FrameVector(desiredChestYawPitchRoll.getReferenceFrame());
         
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            desiredChestYawPitchRoll.getFrameOrientationIncludingFrame(tempOrientation);
            chestOrientationManager.setDesireds(tempOrientation, zeroFrameVector, zeroFrameVector);
         }
      });
   }

   public void initializeContactStates(SideDependentList<Boolean> areHandsInContact, SideDependentList<Boolean> areFeetInContact)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         initializeHandsInContact.get(robotSide).setValue(areHandsInContact.get(robotSide));
         initializeFeetInContact.get(robotSide).setValue(areFeetInContact.get(robotSide));
      }
   }
   
   public void initialize()
   {
      super.initialize();

      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);
      FramePoint currentCoM = new FramePoint(momentumBasedController.getCenterOfMassFrame());
      currentCoM.changeFrame(desiredCoMPosition.getReferenceFrame());
      desiredCoMPosition.set(currentCoM);

      pelvisOrientationManager.setToHoldCurrentInWorldFrame();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         if (initializeHandsInContact.get(robotSide).booleanValue())
         {
            HandLoadBearingPacket handLoadBearingPacket = new HandLoadBearingPacket(robotSide, true);
            DesiredHandLoadBearingProvider desiredHandLoadBearingProvider = (DesiredHandLoadBearingProvider) variousWalkingProviders.getDesiredHandLoadBearingProvider();
            desiredHandLoadBearingProvider.receivedPacket(handLoadBearingPacket);
         }

         if (initializeFeetInContact.get(robotSide).booleanValue())
         {
            feetManager.setFlatFootContactState(robotSide);
         }
         else
         {
            double defaultSwingTime = this.walkingControllerParameters.getDefaultSwingTime();
            feetManager.requestMoveStraight(robotSide, new FramePose(feet.get(robotSide).getFrameAfterParentJoint()), defaultSwingTime);
         }
      }
   }

   protected void doCoMControl()
   {
      momentumRateOfChangeControlModule.setDesiredCoMPosition(desiredCoMPosition);

      momentumRateOfChangeControlModule.compute();
      momentumRateOfChangeControlModule.getMomentumRateOfChange(momentumRateOfChangeData);
      momentumBasedController.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
   }

   protected void doFootControl()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);

         if (footPoseProvider != null && footPoseProvider.checkForNewPose(robotSide))
         {
            FramePose newFootPose = footPoseProvider.getDesiredFootPose(robotSide);
            desiredConfigurationProviders.get(foot).set(newFootPose);
            double defaultSwingTime = this.walkingControllerParameters.getDefaultSwingTime();
            feetManager.requestMoveStraight(robotSide, newFootPose, defaultSwingTime);
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
}
