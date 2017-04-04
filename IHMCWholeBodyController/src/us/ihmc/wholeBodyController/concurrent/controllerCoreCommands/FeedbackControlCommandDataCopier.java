package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FeedbackControlCommandDataCopier
{
   private static final int INITIAL_SIZE = 20;

   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final RecyclingArrayList<JointspaceFeedbackControlCommand> jointspaceFeedbackControlCommands = new RecyclingArrayList<>(INITIAL_SIZE, JointspaceFeedbackControlCommand.class);
   private final RecyclingArrayList<OrientationFeedbackControlCommand> orientationFeedbackControlCommands = new RecyclingArrayList<>(INITIAL_SIZE, OrientationFeedbackControlCommand.class);
   private final RecyclingArrayList<PointFeedbackControlCommand> pointFeedbackControlCommands = new RecyclingArrayList<>(INITIAL_SIZE, PointFeedbackControlCommand.class);
   private final RecyclingArrayList<SpatialFeedbackControlCommand> spatialFeedbackControlCommands = new RecyclingArrayList<>(INITIAL_SIZE, SpatialFeedbackControlCommand.class);

   public FeedbackControlCommandDataCopier()
   {
      clear();
   }

   public void retrieveRigidBodiesFromName(Map<String, RigidBody> nameToRigidBodyMap)
   {
      for (int i = 0; i < orientationFeedbackControlCommands.size(); i++)
      {
         SpatialAccelerationCommand command = orientationFeedbackControlCommands.get(i).getSpatialAccelerationCommand();
         RigidBody base = nameToRigidBodyMap.get(command.getBaseName());
         RigidBody endEffector = nameToRigidBodyMap.get(command.getEndEffectorName());
         command.set(base, endEffector);
      }

      for (int i = 0; i < pointFeedbackControlCommands.size(); i++)
      {
         PointAccelerationCommand command = pointFeedbackControlCommands.get(i).getPointAccelerationCommand();
         RigidBody base = nameToRigidBodyMap.get(command.getBaseName());
         RigidBody endEffector = nameToRigidBodyMap.get(command.getEndEffectorName());
         command.set(base, endEffector);
      }

      for (int i = 0; i < spatialFeedbackControlCommands.size(); i++)
      {
         SpatialAccelerationCommand command = spatialFeedbackControlCommands.get(i).getSpatialAccelerationCommand();
         RigidBody base = nameToRigidBodyMap.get(command.getBaseName());
         RigidBody endEffector = nameToRigidBodyMap.get(command.getEndEffectorName());
         command.set(base, endEffector);
      }
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < jointspaceFeedbackControlCommands.size(); i++)
      {
         JointspaceFeedbackControlCommand command = jointspaceFeedbackControlCommands.get(i);
         command.retrieveJointsFromName(nameToJointMap);
      }
   }

   public void copyFromOther(FeedbackControlCommandList other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfCommands(); i++)
      {
         FeedbackControlCommand<?> commandToCopy = other.getCommand(i);

         switch (commandToCopy.getCommandType())
         {
         case JOINTSPACE:
            copyJointspaceFeedbackControlCommand((JointspaceFeedbackControlCommand) commandToCopy);
            break;
         case ORIENTATION:
            copyOrientationFeedbackControlCommand((OrientationFeedbackControlCommand) commandToCopy);
            break;
         case POINT:
            copyPointFeedbackControlCommand((PointFeedbackControlCommand) commandToCopy);
            break;
         case TASKSPACE:
            copySpatialFeedbackControlCommand((SpatialFeedbackControlCommand) commandToCopy);
         default:
            throw new RuntimeException("The command type: " + commandToCopy.getCommandType() + " is not handled.");
         }
      }
   }

   private void clear()
   {
      feedbackControlCommandList.clear();
      jointspaceFeedbackControlCommands.clear();
      orientationFeedbackControlCommands.clear();
      pointFeedbackControlCommands.clear();
      spatialFeedbackControlCommands.clear();
   }

   private void copyJointspaceFeedbackControlCommand(JointspaceFeedbackControlCommand commandToCopy)
   {
      JointspaceFeedbackControlCommand localCommand = jointspaceFeedbackControlCommands.add();
      localCommand.set(commandToCopy);
      feedbackControlCommandList.addCommand(localCommand);
   }

   private void copyOrientationFeedbackControlCommand(OrientationFeedbackControlCommand commandToCopy)
   {
      OrientationFeedbackControlCommand localCommand = orientationFeedbackControlCommands.add();
      localCommand.set(commandToCopy);
      feedbackControlCommandList.addCommand(localCommand);
   }
   
   private void copyPointFeedbackControlCommand(PointFeedbackControlCommand commandToCopy)
   {
      PointFeedbackControlCommand localCommand = pointFeedbackControlCommands.add();
      localCommand.set(commandToCopy);
      feedbackControlCommandList.addCommand(localCommand);
   }
   
   private void copySpatialFeedbackControlCommand(SpatialFeedbackControlCommand commandToCopy)
   {
      SpatialFeedbackControlCommand localCommand = spatialFeedbackControlCommands.add();
      localCommand.set(commandToCopy);
      feedbackControlCommandList.addCommand(localCommand);
   }

   public FeedbackControlCommandList getFeedbackControlCommandList()
   {
      return feedbackControlCommandList;
   }
}
