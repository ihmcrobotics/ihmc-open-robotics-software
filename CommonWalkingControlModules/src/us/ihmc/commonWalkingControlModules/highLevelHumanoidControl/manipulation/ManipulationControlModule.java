package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.SE3PDGains;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.PipeLine;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.tasks.HandLoadBearingTask;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.tasks.HandPoseTask;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packets.HandPosePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

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

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
   private final SideDependentList<IndividualHandControlModule> individualHandControlModules;

   private final PipeLine<RobotSide> pipeline = new PipeLine<RobotSide>();

   private final ArmControllerParameters armControlParameters;
   private final FullRobotModel fullRobotModel;

   private final HandPoseProvider handPoseProvider;
   private final DesiredHandLoadBearingProvider handLoadBearingProvider;

   private final SE3PDGains taskspaceControlGains = new SE3PDGains();

   public ManipulationControlModule(VariousWalkingProviders variousWalkingProviders, ArmControllerParameters armControlParameters,
         MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      SideDependentList<Integer> jacobianIds = new SideDependentList<Integer>();
      SideDependentList<RigidBody> endEffectors = new SideDependentList<RigidBody>();

      fullRobotModel = momentumBasedController.getFullRobotModel();
      this.armControlParameters = armControlParameters;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody endEffector = fullRobotModel.getHand(robotSide);
         endEffectors.put(robotSide, endEffector);

         int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(fullRobotModel.getChest(), endEffector, endEffector.getBodyFixedFrame());
         jacobianIds.put(robotSide, jacobianId);
      }

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      createFrameVisualizers(dynamicGraphicObjectsListRegistry, fullRobotModel, "midHandPositionControlFrames", false);

      handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      handLoadBearingProvider = variousWalkingProviders.getDesiredHandLoadBearingProvider();

      individualHandControlModules = createIndividualHandControlModules(momentumBasedController, jacobianIds, armControlParameters,
            variousWalkingProviders.getControlStatusProducer());

      kpArmTaskspace = new DoubleYoVariable("kpArmTaskspace", registry);
      kpArmTaskspace.set(armControlParameters.getArmTaskspaceKp());

      zetaArmTaskspace = new DoubleYoVariable("zetaArmTaskspace", registry);
      zetaArmTaskspace.set(armControlParameters.getArmTaskspaceZeta());

      kdArmTaskspace = new DoubleYoVariable("kdArmTaskspace", registry);

      kiArmTaskspace = new DoubleYoVariable("kiArmTaskspace", registry);
      kiArmTaskspace.set(armControlParameters.getArmTaskspaceKi());

      maxIntegralErrorArmTaskspace = new DoubleYoVariable("maxIntegralErrorArmTaskspace", registry);
      maxIntegralErrorArmTaskspace.set(armControlParameters.getArmTaskspaceMaxIntegralError());

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

            taskspaceControlGains.set(kpArmTaskspace.getDoubleValue(), zetaArmTaskspace.getDoubleValue(), kiArmTaskspace.getDoubleValue(),
                  maxIntegralErrorArmTaskspace.getDoubleValue(), kpArmTaskspace.getDoubleValue(), zetaArmTaskspace.getDoubleValue(),
                  kiArmTaskspace.getDoubleValue(), maxIntegralErrorArmTaskspace.getDoubleValue());
         }
      };

      kpArmTaskspace.addVariableChangedListener(listener);
      zetaArmTaskspace.addVariableChangedListener(listener);
      kdArmTaskspace.addVariableChangedListener(listener);
      kiArmTaskspace.addVariableChangedListener(listener);
      maxIntegralErrorArmTaskspace.addVariableChangedListener(listener);

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

   private SideDependentList<IndividualHandControlModule> createIndividualHandControlModules(MomentumBasedController momentumBasedController,
         SideDependentList<Integer> jacobianIds, ArmControllerParameters armControlParameters, ControlStatusProducer controlStatusProducer)
   {
      SideDependentList<IndividualHandControlModule> individualHandControlModules = new SideDependentList<IndividualHandControlModule>();

      for (RobotSide robotSide : RobotSide.values)
      {

         int jacobianId = jacobianIds.get(robotSide);

         IndividualHandControlModule individualHandControlModule = new IndividualHandControlModule(robotSide, taskspaceControlGains, momentumBasedController,
               jacobianId, armControlParameters, controlStatusProducer, registry);
         individualHandControlModules.put(robotSide, individualHandControlModule);
      }

      return individualHandControlModules;
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

      RigidBody base = fullRobotModel.getChest(); // TODO: should use fullRobotModel.getElevator(), but OldMomentumControlModule doesn't support this;

      for (RobotSide robotSide : RobotSide.values)
      {
         handleDefaultState(robotSide);

         handleHandPoses(base, robotSide);

         handleLoadBearing(base, robotSide);
      }
      pipeline.doControl();

      for (RobotSide robotSide : RobotSide.values)
      {
         individualHandControlModules.get(robotSide).doControl();
      }
   }

   private void handleDefaultState(RobotSide robotSide)
   {
      if (handPoseProvider.checkForHomePosition(robotSide))
      {
         goToDefaultState(robotSide, handPoseProvider.getTrajectoryTime());
      }
   }

   private void handleHandPoses(RigidBody base, RobotSide robotSide)
   {
      if (handPoseProvider.checkForNewPose(robotSide))
      {
         if (handPoseProvider.checkPacketDataType(robotSide) == HandPosePacket.DataType.HAND_POSE)
         {
            ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
            HandPoseTask handPoseTask = new HandPoseTask(handPoseProvider.getDesiredHandPose(robotSide), base, handControlFrame,
                  handPoseProvider.getDesiredReferenceFrame(robotSide), individualHandControlModules.get(robotSide), handPoseProvider.getTrajectoryTime(),
                  taskspaceControlGains);
            pipeline.clear(robotSide);
            pipeline.submit(robotSide, handPoseTask);

         }
         else
         {
            individualHandControlModules.get(robotSide).moveUsingQuinticSplines(handPoseProvider.getFinalDesiredJointAngleMaps(robotSide),
                  handPoseProvider.getTrajectoryTime());
         }

      }
   }

   private void handleLoadBearing(RigidBody base, RobotSide robotSide)
   {
      if ((handLoadBearingProvider != null) && handLoadBearingProvider.checkForNewInformation(robotSide))
      {
         if (handLoadBearingProvider.hasLoadBearingBeenRequested(robotSide))
         {
            pipeline.clear(robotSide);

            HandLoadBearingTask handLoadBearingTask = new HandLoadBearingTask(individualHandControlModules.get(robotSide));
            pipeline.submit(robotSide, handLoadBearingTask);
         }
         else
         {
            individualHandControlModules.get(robotSide).holdPositionInBase();
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
      individualHandControlModules.get(robotSide).moveUsingQuinticSplines(armControlParameters.getDefaultArmJointPositions(fullRobotModel, robotSide),
            trajectoryTime);
   }

   public void prepareForLocomotion()
   {
      for (IndividualHandControlModule individualHandControlModule : individualHandControlModules)
      {
         if (individualHandControlModule.isControllingPoseInWorld())
         {
            if (ManipulationControlModule.HOLD_POSE_IN_JOINT_SPACE_WHEN_PREPARE_FOR_LOCOMOTION)
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
         if (individualHandControlModules.get(robotSide).isLoadBearing())
            return true;
      }
      return false;
   }
}
