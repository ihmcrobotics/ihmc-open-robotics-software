package us.ihmc.valkyrie.hands.psyonic;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.IDLSequence.Byte;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.valkyrie.hands.ValkyrieHandController;
import us.ihmc.valkyrie.hands.psyonic.PsyonicHandModel.PsyonicJointName;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage;

public class SimulatedPsyonicController implements ValkyrieHandController
{
   private final YoRegistry registry;

   private final DoubleProvider yoTime;
   private final AtomicReference<PsyonicTrajectoryMessage> trajectoryMessageReference = new AtomicReference<>();
   private final List<JointHandler> jointHandlers = new ArrayList<>();

   public SimulatedPsyonicController(RobotSide robotSide,
                                     FullHumanoidRobotModel fullRobotModel,
                                     JointDesiredOutputListBasics jointDesiredOutputList,
                                     DoubleProvider yoTime,
                                     RealtimeROS2Node realtimeROS2Node,
                                     ROS2Topic<?> inputTopic)
   {
      this.yoTime = yoTime;

      registry = new YoRegistry(robotSide.getCamelCaseName() + getClass().getSimpleName());

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, PsyonicTrajectoryMessage.class, inputTopic, s ->
      {
         trajectoryMessageReference.set(s.takeNextData());
      });

      for (PsyonicJointName jointName : PsyonicJointName.values)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(jointName.getJointName(robotSide));
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         jointHandlers.add(new JointHandler(robotSide, jointName, joint, jointDesiredOutput, registry));
      }
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < jointHandlers.size(); i++)
      {
         jointHandlers.get(i).initialize();
      }
   }

   @Override
   public void doControl()
   {
      PsyonicTrajectoryMessage newMessage = trajectoryMessageReference.getAndSet(null);
      if (newMessage != null)
         handleTrajectoryMessage(newMessage);

      for (int i = 0; i < jointHandlers.size(); i++)
      {
         jointHandlers.get(i).doControl(yoTime.getValue());
      }
   }

   private void handleTrajectoryMessage(PsyonicTrajectoryMessage message)
   {
      Byte jointNames = message.getJointNames();
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = message.getJointspaceTrajectory().getJointTrajectoryMessages();

      if (jointNames.size() != jointTrajectoryMessages.size())
         LogTools.error("Size of jointNames ({}) and jointTrajectoryMessages ({}) is inconsistent", jointNames.size(), jointTrajectoryMessages.size());

      for (int i = 0; i < jointNames.size(); i++)
      {
         JointHandler jointHandler = jointHandlers.get(jointNames.get(i));
         jointHandler.handleTrajectoryMessage(yoTime.getValue(), jointTrajectoryMessages.get(i));
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private static class JointHandler
   {
      private final OneDoFJointBasics joint;
      private final JointDesiredOutputBasics jointDesiredOutput;
      private final MultipleWaypointsTrajectoryGenerator trajectory;

      private final YoDouble trajectoryStartTime;

      public JointHandler(RobotSide robotSide,
                          PsyonicJointName jointName,
                          OneDoFJointBasics joint,
                          JointDesiredOutputBasics jointDesiredOutput,
                          YoRegistry registry)
      {
         this.joint = joint;
         this.jointDesiredOutput = jointDesiredOutput;

         trajectory = new MultipleWaypointsTrajectoryGenerator(jointName.getCamelCaseJointName(robotSide), 5, registry);
         trajectoryStartTime = new YoDouble(jointName.getCamelCaseJointName(robotSide) + "TrajectoryStartTime", registry);
      }

      public void handleTrajectoryMessage(double time, OneDoFJointTrajectoryMessage trajectoryMessage)
      {
         double q_0 = trajectory.getValue();
         double qd_0 = trajectory.getVelocity();
         trajectory.clear();

         Object<TrajectoryPoint1DMessage> trajectoryPoints = trajectoryMessage.getTrajectoryPoints();
         TrajectoryPoint1DMessage firstTrajectoryPoint = trajectoryPoints.getFirst();
         if (firstTrajectoryPoint.getTime() < 1.0e-2)
         {
            trajectory.appendWaypoint(0, firstTrajectoryPoint.getPosition(), firstTrajectoryPoint.getVelocity());

            for (int i = 1; i < trajectoryPoints.size(); i++)
            {
               TrajectoryPoint1DMessage trajectoryPoint = trajectoryPoints.get(i);
               trajectory.appendWaypoint(trajectoryPoint.getTime(), trajectoryPoint.getPosition(), trajectoryPoint.getVelocity());
            }
         }
         else
         {
            trajectory.appendWaypoint(0, q_0, qd_0);

            for (int i = 0; i < trajectoryPoints.size(); i++)
            {
               TrajectoryPoint1DMessage trajectoryPoint = trajectoryPoints.get(i);
               trajectory.appendWaypoint(trajectoryPoint.getTime(), trajectoryPoint.getPosition(), trajectoryPoint.getVelocity());
            }
         }

         trajectory.initialize();
         trajectoryStartTime.set(time);
      }

      public void initialize()
      {
         trajectory.clear();
         trajectory.appendWaypoint(0, joint.getQ(), 0);
         trajectory.initialize();
         trajectoryStartTime.set(0);
      }

      public void doControl(double time)
      {
         trajectory.compute(Math.max(0, time - trajectoryStartTime.getValue()));
         jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
         jointDesiredOutput.setDesiredPosition(trajectory.getValue());
         jointDesiredOutput.setDesiredVelocity(trajectory.getValue());
      }
   }
}
