package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.SE3PIDGains;
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
import com.yobotics.simulationconstructionset.util.GainCalculator;
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

   private final DoubleYoVariable kpArmTaskspace, kdArmTaskspace, kiArmTaskspace, zetaArmTaskspace, maxIntegralErrorArmTaskspace;
   private final DoubleYoVariable maxAccelerationArmTaskspace, maxJerkArmTaskspace;

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
   private final SideDependentList<HandControlModule> handControlModules;

   private final ArmControllerParameters armControlParameters;
   private final FullRobotModel fullRobotModel;

   private final HandPoseProvider handPoseProvider;
   private final DesiredHandLoadBearingProvider handLoadBearingProvider;

   private final SE3PIDGains taskspaceControlGains = new SE3PIDGains();

   public ManipulationControlModule(VariousWalkingProviders variousWalkingProviders, ArmControllerParameters armControlParameters,
         MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      fullRobotModel = momentumBasedController.getFullRobotModel();
      this.armControlParameters = armControlParameters;

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      createFrameVisualizers(dynamicGraphicObjectsListRegistry, fullRobotModel, "HandControlFrames", false);

      handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      handLoadBearingProvider = variousWalkingProviders.getDesiredHandLoadBearingProvider();

      handControlModules = new SideDependentList<HandControlModule>();

      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlModule individualHandControlModule = new HandControlModule(robotSide, taskspaceControlGains, momentumBasedController, armControlParameters,
               variousWalkingProviders.getControlStatusProducer(), registry);
         handControlModules.put(robotSide, individualHandControlModule);
      }

      String suffix = "ArmTaskspace";
      
      kpArmTaskspace = new DoubleYoVariable("kp" + suffix, registry);
      kpArmTaskspace.set(armControlParameters.getArmTaskspaceKp());

      zetaArmTaskspace = new DoubleYoVariable("zeta" + suffix, registry);
      zetaArmTaskspace.set(armControlParameters.getArmTaskspaceZeta());

      kdArmTaskspace = new DoubleYoVariable("kd" + suffix, registry);

      kiArmTaskspace = new DoubleYoVariable("ki" + suffix, registry);
      kiArmTaskspace.set(armControlParameters.getArmTaskspaceKi());

      maxIntegralErrorArmTaskspace = new DoubleYoVariable("maxIntegralError" + suffix, registry);
      maxIntegralErrorArmTaskspace.set(armControlParameters.getArmTaskspaceMaxIntegralError());

      maxAccelerationArmTaskspace = new DoubleYoVariable("maxAcceleration" + suffix, registry);
      maxAccelerationArmTaskspace.set(armControlParameters.getArmTaskspaceMaxAcceleration());

      maxJerkArmTaskspace = new DoubleYoVariable("maxJerk" + suffix, registry);
      maxJerkArmTaskspace.set(armControlParameters.getArmTaskspaceMaxJerk());

      setupVariableListener();

      parentRegistry.addChild(registry);
   }

   private void setupVariableListener()
   {
      VariableChangedListener listener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            kdArmTaskspace.set(GainCalculator.computeDerivativeGain(kpArmTaskspace.getDoubleValue(), zetaArmTaskspace.getDoubleValue()));

            taskspaceControlGains.setPositionGains(kpArmTaskspace.getDoubleValue(), zetaArmTaskspace.getDoubleValue(), kiArmTaskspace.getDoubleValue(), maxIntegralErrorArmTaskspace.getDoubleValue());
            taskspaceControlGains.setOrientationGains(kpArmTaskspace.getDoubleValue(), zetaArmTaskspace.getDoubleValue(), kiArmTaskspace.getDoubleValue(), maxIntegralErrorArmTaskspace.getDoubleValue());
            taskspaceControlGains.setMaximumAccelerationAndJerk(maxAccelerationArmTaskspace.getDoubleValue(), maxJerkArmTaskspace.getDoubleValue());
         }
      };

      kpArmTaskspace.addVariableChangedListener(listener);
      zetaArmTaskspace.addVariableChangedListener(listener);
      kdArmTaskspace.addVariableChangedListener(listener);
      kiArmTaskspace.addVariableChangedListener(listener);
      maxIntegralErrorArmTaskspace.addVariableChangedListener(listener);
      maxAccelerationArmTaskspace.addVariableChangedListener(listener);
      maxJerkArmTaskspace.addVariableChangedListener(listener);

      listener.variableChanged(null);
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
            handControlModules.get(robotSide).moveInStraightLine(handPoseProvider.getDesiredHandPose(robotSide), handPoseProvider.getTrajectoryTime(), handPoseProvider.getDesiredReferenceFrame(robotSide), taskspaceControlGains);
         }
         else
         {
            handControlModules.get(robotSide).moveUsingQuinticSplines(handPoseProvider.getFinalDesiredJointAngleMaps(robotSide), handPoseProvider.getTrajectoryTime());
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
      for (HandControlModule individualHandControlModule : handControlModules)
      {
         if (individualHandControlModule.isControllingPoseInWorld())
         {
            if (HOLD_POSE_IN_JOINT_SPACE_WHEN_PREPARE_FOR_LOCOMOTION)
               individualHandControlModule.holdPositionInJointSpace();
            else
               individualHandControlModule.holdPositionInBase();
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
