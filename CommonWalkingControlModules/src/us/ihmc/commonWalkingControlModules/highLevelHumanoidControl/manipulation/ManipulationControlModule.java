package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ObjectWeightProvider;
import us.ihmc.commonWalkingControlModules.sensors.ProvidedMassMatrixToolRigidBody;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
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
   public static final boolean HOLD_POSE_IN_JOINT_SPACE = true;
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
   private final HandComplianceControlParametersProvider handComplianceControlParametersProvider;

   private final ObjectWeightProvider objectWeightProvider;
   private final SideDependentList<ProvidedMassMatrixToolRigidBody> toolRigidBodies;

   private final DoubleYoVariable handSwingClearance = new DoubleYoVariable("handSwingClearance", registry);

   private final DoubleYoVariable timeTransitionBeforeLoadBearing = new DoubleYoVariable("timeTransitionBeforeLoadBearing", registry);

   private final BooleanYoVariable goToLoadBearingWhenHandlingHandstep;

   private final BooleanYoVariable isIgnoringInputs = new BooleanYoVariable("isManipulationIgnoringInputs", registry);
   private final DoubleYoVariable startTimeForIgnoringInputs = new DoubleYoVariable("startTimeForIgnoringManipulationInputs", registry);
   private final DoubleYoVariable durationForIgnoringInputs = new DoubleYoVariable("durationForIgnoringManipulationInputs", registry);

   private final DoubleYoVariable yoTime;

   public ManipulationControlModule(VariousWalkingProviders variousWalkingProviders, ArmControllerParameters armControllerParameters,
         MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      fullRobotModel = momentumBasedController.getFullRobotModel();
      this.armControlParameters = armControllerParameters;
      this.yoTime = momentumBasedController.getYoTime();

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      createFrameVisualizers(yoGraphicsListRegistry, fullRobotModel, "HandControlFrames", true);

      handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      handstepProvider = variousWalkingProviders.getHandstepProvider();
      handLoadBearingProvider = variousWalkingProviders.getDesiredHandLoadBearingProvider();
      handComplianceControlParametersProvider = variousWalkingProviders.getHandComplianceControlParametersProvider();

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

      if (yoTime.getDoubleValue() - startTimeForIgnoringInputs.getDoubleValue() < durationForIgnoringInputs.getDoubleValue())
      {
         isIgnoringInputs.set(true);
         handPoseProvider.clear();
      }
      else
      {
         isIgnoringInputs.set(false);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         handleCompliantControlRequests(robotSide);

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

      if (objectWeightProvider != null && objectWeightProvider.isNewInformationAvailable())
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
            FramePose desiredHandPose = handPoseProvider.getDesiredHandPose(robotSide);
            double trajectoryTime = handPoseProvider.getTrajectoryTime();
            ReferenceFrame desiredReferenceFrame = handPoseProvider.getDesiredReferenceFrame(robotSide);
            boolean[] controlledOrientationAxes = handPoseProvider.getControlledOrientationAxes(robotSide);
            double percentOfTrajectoryWithOrientationBeingControlled = handPoseProvider.getPercentOfTrajectoryWithOrientationBeingControlled(robotSide);

            handControlModules.get(robotSide).moveInStraightLine(desiredHandPose, trajectoryTime, desiredReferenceFrame, controlledOrientationAxes,
                  percentOfTrajectoryWithOrientationBeingControlled, handSwingClearance.getDoubleValue());
         }
         else
         {
            Map<OneDoFJoint, Double> finalDesiredJointAngleMaps = handPoseProvider.getFinalDesiredJointAngleMaps(robotSide);
            double trajectoryTime = handPoseProvider.getTrajectoryTime();
            handControlModules.get(robotSide).moveUsingQuinticSplines(finalDesiredJointAngleMaps, trajectoryTime);
         }
      }
      else if (handPoseProvider.checkForNewPoseList(robotSide))
      {
         if (handPoseProvider.checkHandPoseListPacketDataType(robotSide) == HandPosePacket.DataType.HAND_POSE)
         {
            FramePose[] desiredHandPoses = handPoseProvider.getDesiredHandPoses(robotSide);
            double trajectoryTime = handPoseProvider.getTrajectoryTime();
            ReferenceFrame desiredReferenceFrame = handPoseProvider.getDesiredReferenceFrame(robotSide);

            handControlModules.get(robotSide).moveInStraightLinesViaWayPoints(desiredHandPoses, trajectoryTime, desiredReferenceFrame);
         }
         else
         {
            Map<OneDoFJoint, double[]> desiredJointAngleForWaypointTrajectory = handPoseProvider.getDesiredJointAngleForWaypointTrajectory(robotSide);
            double trajectoryTime = handPoseProvider.getTrajectoryTime();

            handControlModules.get(robotSide).moveJointspaceWithWaypoints(desiredJointAngleForWaypointTrajectory, trajectoryTime);
         }
      }
      else if (handPoseProvider.checkForNewRotateAboutAxisPacket(robotSide))
      {
         Point3d rotationAxisOriginInWorld = handPoseProvider.getRotationAxisOriginInWorld(robotSide);
         Vector3d rotationAxisInWorld = handPoseProvider.getRotationAxisInWorld(robotSide);
         double rotationAngleRightHandRule = handPoseProvider.getRotationAngleRightHandRule(robotSide);
         boolean controlHandAngleAboutAxis = handPoseProvider.controlHandAngleAboutAxis(robotSide);
         double graspOffsetFromControlFrame = handPoseProvider.getGraspOffsetFromControlFrame(robotSide);
         double trajectoryTime = handPoseProvider.getTrajectoryTime();

         if(handPoseProvider.checkHandRotateAboutAxisDataType(robotSide) == HandRotateAboutAxisPacket.DataType.ROTATE_ABOUT_AXIS_FORCE_CONTROLLED)
         {
        	 Vector3d forceConstraintVector = handPoseProvider.getForceConstraint(robotSide);
        	 double desiredTangentialForce = handPoseProvider.getTangentialForce(robotSide);
        	 handControlModules.get(robotSide).moveInCircleUsingForceControl(rotationAxisOriginInWorld, rotationAxisInWorld, rotationAngleRightHandRule, controlHandAngleAboutAxis,
        			 graspOffsetFromControlFrame, trajectoryTime, forceConstraintVector, desiredTangentialForce);
         }
         else
         {
            handControlModules.get(robotSide).moveInCircle(rotationAxisOriginInWorld, rotationAxisInWorld, rotationAngleRightHandRule, controlHandAngleAboutAxis, graspOffsetFromControlFrame, trajectoryTime);
         }
         
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
      if (handPoseProvider != null)
      {
         if (handPoseProvider.checkAndResetStopCommand(robotSide))
         {
            handControlModules.get(robotSide).holdPositionInJointSpace();
         }
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

   private void handleCompliantControlRequests(RobotSide robotSide)
   {
      if (handComplianceControlParametersProvider != null && handComplianceControlParametersProvider.checkForNewRequest(robotSide))
      {
         if (handComplianceControlParametersProvider.isResetRequested(robotSide))
         {
            handControlModules.get(robotSide).setEnableCompliantControl(false, null, null, null, null, Double.NaN, Double.NaN);
         }
         else
         {
            boolean[] enableLinearCompliance = handComplianceControlParametersProvider.getEnableLinearCompliance(robotSide);
            boolean[] enableAngularCompliance = handComplianceControlParametersProvider.getEnableAngularCompliance(robotSide);
            Vector3d desiredForce = handComplianceControlParametersProvider.getDesiredForce(robotSide);
            Vector3d desiredTorque = handComplianceControlParametersProvider.getDesiredTorque(robotSide);
            double forceDeadzone = handComplianceControlParametersProvider.getForceDeadzone(robotSide);
            double torqueDeadzone = handComplianceControlParametersProvider.getTorqueDeadzone(robotSide);
            handControlModules.get(robotSide).setEnableCompliantControl(true, enableLinearCompliance, enableAngularCompliance, desiredForce, desiredTorque, forceDeadzone, torqueDeadzone);
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

   public void initializeDesiredToCurrent()
   {
      hasBeenInitialized.set(true);
      for (RobotSide side : RobotSide.values)
      {
         handControlModules.get(side).initializeDesiredToCurrent();
      }
   }

   public void prepareForLocomotion()
   {
      holdCurrentArmConfiguration();
   }

   public void freeze()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlModule handControlModule = handControlModules.get(robotSide);
         handControlModule.holdPositionInJointSpace();
         handControlModule.resetJointIntegrators();
      }
   }

   public void ignoreInputsForGivenDuration(double duration)
   {
      startTimeForIgnoringInputs.set(yoTime.getDoubleValue());
      durationForIgnoringInputs.set(duration);
   }

   public void holdCurrentArmConfiguration()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         holdArmCurrentConfiguration(handControlModules.get(robotSide));
      }
   }

   private void holdArmCurrentConfiguration(HandControlModule handControlModule)
   {
      if (handControlModule.isControllingPoseInWorld())
      {
         if (HOLD_POSE_IN_JOINT_SPACE)
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
