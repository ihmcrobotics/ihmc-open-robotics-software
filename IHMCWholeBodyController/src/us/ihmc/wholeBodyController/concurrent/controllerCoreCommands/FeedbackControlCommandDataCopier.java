package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.SpatialFeedbackControlCommand;
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
         OrientationFeedbackControlCommand command = orientationFeedbackControlCommands.get(i);
         command.setBase(nameToRigidBodyMap.get(command.getBaseName()));
         command.setEndEffector(nameToRigidBodyMap.get(command.getEndEffectorName()));
      }

      for (int i = 0; i < pointFeedbackControlCommands.size(); i++)
      {
         PointFeedbackControlCommand command = pointFeedbackControlCommands.get(i);
         command.setBase(nameToRigidBodyMap.get(command.getBaseName()));
         command.setEndEffector(nameToRigidBodyMap.get(command.getEndEffectorName()));
      }

      for (int i = 0; i < spatialFeedbackControlCommands.size(); i++)
      {
         SpatialFeedbackControlCommand command = spatialFeedbackControlCommands.get(i);
         command.setBase(nameToRigidBodyMap.get(command.getBaseName()));
         command.setEndEffector(nameToRigidBodyMap.get(command.getEndEffectorName()));
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
         case JOINTSPACE_CONTROL:
            copyJointspaceFeedbackControlCommand((JointspaceFeedbackControlCommand) commandToCopy);
            break;
         case ORIENTATION_CONTROL:
            copyOrientationFeedbackControlCommand((OrientationFeedbackControlCommand) commandToCopy);
            break;
         case POINT_CONTROL:
            copyPointFeedbackControlCommand((PointFeedbackControlCommand) commandToCopy);
            break;
         case SPATIAL_CONTROL:
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
