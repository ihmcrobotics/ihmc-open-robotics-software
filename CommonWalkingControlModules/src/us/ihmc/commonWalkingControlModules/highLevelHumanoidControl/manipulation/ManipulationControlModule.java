package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ObjectWeightProvider;
import us.ihmc.commonWalkingControlModules.sensors.ProvidedMassMatrixToolRigidBody;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   public static final boolean HOLD_POSE_IN_JOINT_SPACE_WHEN_PREPARE_FOR_LOCOMOTION = true;
   private static final double TO_DEFAULT_CONFIGURATION_TRAJECTORY_TIME = 2.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<YoGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
   private final SideDependentList<HandControlModule> handControlModules;

   private final ArmControllerParameters armControlParameters;
   private final FullRobotModel fullRobotModel;

   private final HandPoseProvider handPoseProvider;
   private final HandstepProvider handstepProvider;
   private final HandLoadBearingProvider handLoadBearingProvider;
   
   private final ObjectWeightProvider objectWeightProvider;
   private final SideDependentList<ProvidedMassMatrixToolRigidBody> toolRigidBodies;

   private final DoubleYoVariable handSwingClearance = new DoubleYoVariable("handSwingClearance", registry);

   private final DoubleYoVariable timeTransitionBeforeLoadBearing = new DoubleYoVariable("timeTransitionBeforeLoadBearing", registry);

   private final BooleanYoVariable goToLoadBearingWhenHandlingHandstep;

   public ManipulationControlModule(VariousWalkingProviders variousWalkingProviders, ArmControllerParameters armControllerParameters,
         MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      fullRobotModel = momentumBasedController.getFullRobotModel();
      this.armControlParameters = armControllerParameters;

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      createFrameVisualizers(yoGraphicsListRegistry, fullRobotModel, "HandControlFrames", true);

      handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      handstepProvider = variousWalkingProviders.getHandstepProvider();
      handLoadBearingProvider = variousWalkingProviders.getDesiredHandLoadBearingProvider();
      
      objectWeightProvider = variousWalkingProviders.getObjectWeightProvider();
      toolRigidBodies = momentumBasedController.getToolRigitBodies();

      handControlModules = new SideDependentList<HandControlModule>();

      YoPIDGains jointspaceControlGains = armControllerParameters.createJointspaceControlGains(registry);
      YoSE3PIDGains taskspaceGains = armControllerParameters.createTaskspaceControlGains(registry);
      YoSE3PIDGains taskspaceLoadBearingGains = armControllerParameters.createTaskspaceControlGainsForLoadBearing(registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlModule individualHandControlModule = new HandControlModule(robotSide, momentumBasedController, armControllerParameters,
               jointspaceControlGains, taskspaceGains, taskspaceLoadBearingGains, variousWalkingProviders.getControlStatusProducer(),
               variousWalkingProviders.getHandPoseStatusProducer(), registry);
         handControlModules.put(robotSide, individualHandControlModule);
      }

      goToLoadBearingWhenHandlingHandstep = new BooleanYoVariable("goToLoadBearingWhenHandlingHandstep", registry);
      goToLoadBearingWhenHandlingHandstep.set(true);

      handSwingClearance.set(0.08);
      timeTransitionBeforeLoadBearing.set(0.2);

      parentRegistry.addChild(registry);
   }

   private void createFrameVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, FullRobotModel fullRobotModel, String listName, boolean enable)
   {
      YoGraphicsList list = new YoGraphicsList(listName);
      if (yoGraphicsListRegistry != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            ReferenceFrame handPositionControlFrame = fullRobotModel.getHandControlFrame(robotSide);
            if (handPositionControlFrame != null)
            {
               YoGraphicReferenceFrame dynamicGraphicReferenceFrame = new YoGraphicReferenceFrame(handPositionControlFrame, registry, 0.1);
               dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
               list.add(dynamicGraphicReferenceFrame);
            }
         }
         yoGraphicsListRegistry.registerYoGraphicsList(list);

         if (!enable)
            list.hideYoGraphics();
      }
   }

   public void initialize()
   {
      doControl();
   }

   public void doControl()
   {
      // Important especially when switching between high level states. In such case, we don't want the arm to go to home position
      if (!hasBeenInitialized.getBooleanValue())
      {
         goToDefaultState();
         hasBeenInitialized.set(true);
      }

      updateGraphics();

      for (RobotSide robotSide : RobotSide.values)
      {
         handleDefaultState(robotSide);

         handleHandPoses(robotSide);
         handleHandPauses(robotSide);
         handleHandsteps(robotSide);
         handleLoadBearing(robotSide);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         handControlModules.get(robotSide).doControl();
      }
      
      if(objectWeightProvider != null && objectWeightProvider.isNewInformationAvailable())
      {
         toolRigidBodies.get(objectWeightProvider.getRobotSide()).setMass(objectWeightProvider.getWeight());
      }
   }

   private void handleDefaultState(RobotSide robotSide)
   {
      if (handPoseProvider == null)
         return;

      if (handPoseProvider.checkForHomePosition(robotSide))
      {
         goToDefaultState(robotSide, handPoseProvider.getTrajectoryTime());
      }
   }

   private void handleHandPoses(RobotSide robotSide)
   {
      if (handPoseProvider == null)
         return;

      if (handPoseProvider.checkForNewPose(robotSide))
      {
         if (handPoseProvider.checkHandPosePacketDataType(robotSide) == HandPosePacket.DataType.HAND_POSE)
         {
            handControlModules.get(robotSide).moveInStraightLine(handPoseProvider.getDesiredHandPose(robotSide), handPoseProvider.getTrajectoryTime(),
                  handPoseProvider.getDesiredReferenceFrame(robotSide), handSwingClearance.getDoubleValue());
         }
         else
         {
            handControlModules.get(robotSide).moveUsingQuinticSplines(handPoseProvider.getFinalDesiredJointAngleMaps(robotSide),
                  handPoseProvider.getTrajectoryTime());
         }
      }
      else if (handPoseProvider.checkForNewPoseList(robotSide))
      {
         if (handPoseProvider.checkHandPoseListPacketDataType(robotSide) == HandPosePacket.DataType.HAND_POSE)
         {
             handControlModules.get(robotSide).moveInStraightLinesViaWayPoints(handPoseProvider.getDesiredHandPoses(robotSide),
                     handPoseProvider.getTrajectoryTime(), handPoseProvider.getDesiredReferenceFrame(robotSide));
         }
         else
         {
            handControlModules.get(robotSide).moveJointspaceWithWaypoints(handPoseProvider.getDesiredJointAngleForWaypointTrajectory(robotSide),
                  handPoseProvider.getTrajectoryTime());
         }
      }
      else if (handPoseProvider.checkForNewRotateAboutAxisPacket(robotSide))
      {
         handControlModules.get(robotSide).moveInCircle(handPoseProvider.getRotationAxisOriginInWorld(robotSide), handPoseProvider.getRotationAxisInWorld(robotSide), handPoseProvider.getRotationAngleRightHandRule(robotSide), handPoseProvider.getTrajectoryTime());
      }
      else if (handPoseProvider.checkForNewArmJointTrajectory(robotSide))
      {
         ArmJointTrajectoryPacket armJointTrajectoryPacket = handPoseProvider.getArmJointTrajectoryPacket(robotSide);
         if (armJointTrajectoryPacket != null)
         {
            handControlModules.get(robotSide).moveUsingCubicTrajectory(armJointTrajectoryPacket);
         }
      }
   }

   private void handleHandPauses(RobotSide robotSide)
   {
      if (handPoseProvider == null)
         return;

      if (handPoseProvider.checkForNewPauseCommand(robotSide))
      {
         handPoseProvider.getPauseCommand(robotSide);
         handControlModules.get(robotSide).holdPositionInJointSpace();
      }
   }

   private void handleHandsteps(RobotSide robotSide)
   {
      if ((handstepProvider != null) && (handstepProvider.checkForNewHandstep(robotSide)))
      {
         Handstep desiredHandstep = handstepProvider.getDesiredHandstep(robotSide);
         FramePose handstepPose = new FramePose(ReferenceFrame.getWorldFrame());
         desiredHandstep.getPose(handstepPose);
         FrameVector surfaceNormal = new FrameVector();
         desiredHandstep.getSurfaceNormal(surfaceNormal);

         ReferenceFrame trajectoryFrame = handstepPose.getReferenceFrame();
         double swingTrajectoryTime = desiredHandstep.getSwingTrajectoryTime();
         handControlModules.get(robotSide).moveTowardsObjectAndGoToSupport(handstepPose, surfaceNormal, handSwingClearance.getDoubleValue(),
               swingTrajectoryTime, trajectoryFrame, goToLoadBearingWhenHandlingHandstep.getBooleanValue(), timeTransitionBeforeLoadBearing.getDoubleValue());
      }
   }

   private void handleLoadBearing(RobotSide robotSide)
   {
      if ((handLoadBearingProvider != null) && handLoadBearingProvider.checkForNewInformation(robotSide))
      {
         if (handLoadBearingProvider.hasLoadBearingBeenRequested(robotSide))
         {
            handControlModules.get(robotSide).requestLoadBearing();
         }
         else
         {
            handControlModules.get(robotSide).holdPositionInBase();
         }
      }
   }

   public void goToDefaultState()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         goToDefaultState(robotSide, TO_DEFAULT_CONFIGURATION_TRAJECTORY_TIME);
      }
   }

   public void goToDefaultState(RobotSide robotSide, double trajectoryTime)
   {
      handControlModules.get(robotSide).moveUsingQuinticSplines(armControlParameters.getDefaultArmJointPositions(fullRobotModel, robotSide), trajectoryTime);
   }

   public void prepareForLocomotion()
   {
      for (HandControlModule handControlModule : handControlModules)
      {
         freeze(handControlModule);
      }
   }

   public void freeze()
   {
      for (HandControlModule handControlModule : handControlModules)
      {
         freeze(handControlModule);
      }
   }

   private void freeze(HandControlModule handControlModule)
   {
      if (handControlModule.isControllingPoseInWorld())
      {
         if (HOLD_POSE_IN_JOINT_SPACE_WHEN_PREPARE_FOR_LOCOMOTION)
            handControlModule.holdPositionInJointSpace();
         else
            handControlModule.holdPositionInBase();
      }
   }

   private void updateGraphics()
   {
      for (int i = 0; i < dynamicGraphicReferenceFrames.size(); i++)
      {
         dynamicGraphicReferenceFrames.get(i).update();
      }
   }

   public boolean isAtLeastOneHandLoadBearing()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handControlModules.get(robotSide).isLoadBearing())
            return true;
      }
      return false;
   }

   public void setHandSwingClearanceForHandsteps(double handSwingClearance)
   {
      this.handSwingClearance.set(handSwingClearance);
   }

   public double getHandSwingClearanceForHandsteps()
   {
      return handSwingClearance.getDoubleValue();
   }
}
