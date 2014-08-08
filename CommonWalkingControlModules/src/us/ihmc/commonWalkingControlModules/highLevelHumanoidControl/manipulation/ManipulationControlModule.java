package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packets.HandPosePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.controller.GainCalculator;
import com.yobotics.simulationconstructionset.util.controller.SE3PIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoPIDGains;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   public static final boolean HOLD_POSE_IN_JOINT_SPACE_WHEN_PREPARE_FOR_LOCOMOTION = true;
   private static final double TO_DEFAULT_CONFIGURATION_TRAJECTORY_TIME = 1.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   private final DoubleYoVariable kpTaskspace, kdTaskspace, kiTaskspace, zetaTaskspace, maxIntegralErrorTaskspace;
   private final DoubleYoVariable maxAccelerationTaskspace, maxJerkTaskspace;

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
   private final SideDependentList<HandControlModule> handControlModules;

   private final ArmControllerParameters armControlParameters;
   private final FullRobotModel fullRobotModel;

   private final HandPoseProvider handPoseProvider;
   private final DesiredHandLoadBearingProvider handLoadBearingProvider;

   private final SE3PIDGains taskspaceControlGains = new SE3PIDGains();

   public ManipulationControlModule(VariousWalkingProviders variousWalkingProviders, ArmControllerParameters armControllerParameters,
         MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      fullRobotModel = momentumBasedController.getFullRobotModel();
      this.armControlParameters = armControllerParameters;

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      createFrameVisualizers(dynamicGraphicObjectsListRegistry, fullRobotModel, "HandControlFrames", false);

      handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      handLoadBearingProvider = variousWalkingProviders.getDesiredHandLoadBearingProvider();

      handControlModules = new SideDependentList<HandControlModule>();

      YoPIDGains jointspaceControlGains = new YoPIDGains("ArmJointspace", registry);
      jointspaceControlGains.setKp(armControllerParameters.getArmJointspaceKp());
      jointspaceControlGains.setZeta(armControllerParameters.getArmJointspaceZeta());
      jointspaceControlGains.setKi(armControllerParameters.getArmJointspaceKi());
      jointspaceControlGains.setMaximumIntegralError(armControllerParameters.getArmJointspaceMaxIntegralError());
      jointspaceControlGains.setMaximumAcceleration(armControllerParameters.getArmJointspaceMaxAcceleration());
      jointspaceControlGains.setMaximumJerk(armControllerParameters.getArmJointspaceMaxJerk());
      jointspaceControlGains.createDerivativeGainUpdater(true);

      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlModule individualHandControlModule = new HandControlModule(robotSide, momentumBasedController, armControllerParameters,
               jointspaceControlGains, variousWalkingProviders.getControlStatusProducer(), registry);
         handControlModules.put(robotSide, individualHandControlModule);
      }

      String taskspaceSuffix = "ArmTaskspace";

      kpTaskspace = new DoubleYoVariable("kp" + taskspaceSuffix, registry);
      kpTaskspace.set(armControllerParameters.getArmTaskspaceKp());

      zetaTaskspace = new DoubleYoVariable("zeta" + taskspaceSuffix, registry);
      zetaTaskspace.set(armControllerParameters.getArmTaskspaceZeta());

      kdTaskspace = new DoubleYoVariable("kd" + taskspaceSuffix, registry);

      kiTaskspace = new DoubleYoVariable("ki" + taskspaceSuffix, registry);
      kiTaskspace.set(armControllerParameters.getArmTaskspaceKi());

      maxIntegralErrorTaskspace = new DoubleYoVariable("maxIntegralError" + taskspaceSuffix, registry);
      maxIntegralErrorTaskspace.set(armControllerParameters.getArmTaskspaceMaxIntegralError());

      maxAccelerationTaskspace = new DoubleYoVariable("maxAcceleration" + taskspaceSuffix, registry);
      maxAccelerationTaskspace.set(armControllerParameters.getArmTaskspaceMaxAcceleration());

      maxJerkTaskspace = new DoubleYoVariable("maxJerk" + taskspaceSuffix, registry);
      maxJerkTaskspace.set(armControllerParameters.getArmTaskspaceMaxJerk());

      setupVariableListener();

      parentRegistry.addChild(registry);
   }

   private void setupVariableListener()
   {
      VariableChangedListener taskspaceListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            kdTaskspace.set(GainCalculator.computeDerivativeGain(kpTaskspace.getDoubleValue(), zetaTaskspace.getDoubleValue()));

            taskspaceControlGains.setPositionGains(kpTaskspace.getDoubleValue(), kdTaskspace.getDoubleValue(), kiTaskspace.getDoubleValue(),
                  maxIntegralErrorTaskspace.getDoubleValue());
            taskspaceControlGains.setOrientationGains(kpTaskspace.getDoubleValue(), kdTaskspace.getDoubleValue(), kiTaskspace.getDoubleValue(),
                  maxIntegralErrorTaskspace.getDoubleValue());
            taskspaceControlGains.setMaximumAccelerationAndJerk(maxAccelerationTaskspace.getDoubleValue(), maxJerkTaskspace.getDoubleValue());
         }
      };

      kpTaskspace.addVariableChangedListener(taskspaceListener);
      zetaTaskspace.addVariableChangedListener(taskspaceListener);
      kdTaskspace.addVariableChangedListener(taskspaceListener);
      kiTaskspace.addVariableChangedListener(taskspaceListener);
      maxIntegralErrorTaskspace.addVariableChangedListener(taskspaceListener);
      maxAccelerationTaskspace.addVariableChangedListener(taskspaceListener);
      maxJerkTaskspace.addVariableChangedListener(taskspaceListener);

      taskspaceListener.variableChanged(null);
   }

   private void createFrameVisualizers(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, FullRobotModel fullRobotModel, String listName,
         boolean enable)
   {
      DynamicGraphicObjectsList list = new DynamicGraphicObjectsList(listName);
      if (dynamicGraphicObjectsListRegistry != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            ReferenceFrame handPositionControlFrame = fullRobotModel.getHandControlFrame(robotSide);
            if (handPositionControlFrame != null)
            {
               DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(handPositionControlFrame, registry, 0.1);
               dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
               list.add(dynamicGraphicReferenceFrame);
            }
         }
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(list);

         if (!enable)
            list.hideDynamicGraphicObjects();
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

         handleLoadBearing(robotSide);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         handControlModules.get(robotSide).doControl();
      }
   }

   private void handleDefaultState(RobotSide robotSide)
   {
      if (handPoseProvider.checkForHomePosition(robotSide))
      {
         goToDefaultState(robotSide, handPoseProvider.getTrajectoryTime());
      }
   }

   private void handleHandPoses(RobotSide robotSide)
   {
      if (handPoseProvider.checkForNewPose(robotSide))
      {
         if (handPoseProvider.checkPacketDataType(robotSide) == HandPosePacket.DataType.HAND_POSE)
         {
            handControlModules.get(robotSide).moveInStraightLine(handPoseProvider.getDesiredHandPose(robotSide), handPoseProvider.getTrajectoryTime(),
                  handPoseProvider.getDesiredReferenceFrame(robotSide), taskspaceControlGains);
         }
         else
         {
            handControlModules.get(robotSide).moveUsingQuinticSplines(handPoseProvider.getFinalDesiredJointAngleMaps(robotSide),
                  handPoseProvider.getTrajectoryTime());
         }
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
            handControlModules.get(robotSide).holdPositionInBase(taskspaceControlGains);
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
      for (HandControlModule individualHandControlModule : handControlModules)
      {
         if (individualHandControlModule.isControllingPoseInWorld())
         {
            if (HOLD_POSE_IN_JOINT_SPACE_WHEN_PREPARE_FOR_LOCOMOTION)
               individualHandControlModule.holdPositionInJointSpace();
            else
               individualHandControlModule.holdPositionInBase(taskspaceControlGains);
         }
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
}
