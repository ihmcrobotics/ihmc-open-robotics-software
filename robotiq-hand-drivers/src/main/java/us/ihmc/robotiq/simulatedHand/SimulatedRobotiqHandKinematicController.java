package us.ihmc.robotiq.simulatedHand;

import us.ihmc.avatar.kinematicsSimulation.SimulatedHandKinematicController;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class SimulatedRobotiqHandKinematicController implements SimulatedHandKinematicController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FullHumanoidRobotModel fullRobotModel;
   private final RealtimeROS2Node realtimeROS2Node;
   private final DoubleProvider controllerTime;

   private final List<OneDoFJointBasics> controlledFingerJoints = new ArrayList<>();
   private final SimulatedRobotiqHandsController controller;

   public SimulatedRobotiqHandKinematicController(String simpleRobotName,
                                                  FullHumanoidRobotModel fullRobotModel,
                                                  RealtimeROS2Node realtimeROS2Node,
                                                  DoubleProvider controllerTime,
                                                  RobotSide[] sides)
   {
      this.fullRobotModel = fullRobotModel;
      this.realtimeROS2Node = realtimeROS2Node;
      this.controllerTime = controllerTime;

      for (RobotSide robotSide : sides)
      {
         SubtreeStreams.fromChildren(OneDoFJointBasics.class, fullRobotModel.getHand(robotSide)).forEach(controlledFingerJoints::add);
      }

      OneDoFJointBasics[] joints = controlledFingerJoints.toArray(new OneDoFJointBasics[0]);
      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(joints);

      controller = new SimulatedRobotiqHandsController(fullRobotModel,
                                                       jointDesiredOutputList,
                                                       controllerTime,
                                                       realtimeROS2Node,
                                                       HumanoidControllerAPI.getOutputTopic(simpleRobotName),
                                                       HumanoidControllerAPI.getInputTopic(simpleRobotName),
                                                       sides,
                                                       JointDesiredControlMode.POSITION);
      registry.addChild(controller.getYoRegistry());
   }

   @Override
   public void initialize()
   {
      controller.initialize();
   }

   @Override
   public void doControl()
   {
      controller.doControl();
   }

   @Override
   public void cleanup()
   {
      controller.cleanup();
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }
}
