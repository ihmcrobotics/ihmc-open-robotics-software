package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class InverseKinematicsCommandDataCopier
{
   private static final int INITIAL_CAPACITY = 20;

   private final InverseKinematicsCommandList inverseKinematicsCommandList = new InverseKinematicsCommandList();

   private final RecyclingArrayList<JointspaceVelocityCommand> jointspaceVelocityCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, JointspaceVelocityCommand.class);
   private final RecyclingArrayList<SpatialVelocityCommand> spatialVelocityCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, SpatialVelocityCommand.class);
   private final RecyclingArrayList<MomentumCommand> momentumCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, MomentumCommand.class);

   public InverseKinematicsCommandDataCopier()
   {
      clear();
   }

   public void retrieveRigidBodiesFromName(Map<String, RigidBody> nameToRigidBodyMap)
   {
      for (int i = 0; i < spatialVelocityCommands.size(); i++)
      {
         SpatialVelocityCommand command = spatialVelocityCommands.get(i);
         RigidBody base = nameToRigidBodyMap.get(command.getBaseName());
         RigidBody endEffector = nameToRigidBodyMap.get(command.getEndEffectorName());
         command.set(base, endEffector);
      }
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < jointspaceVelocityCommands.size(); i++)
      {
         JointspaceVelocityCommand command = jointspaceVelocityCommands.get(i);
         command.retrieveJointsFromName(nameToJointMap);
      }
   }

   public void copyFromOther(InverseKinematicsCommandList other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfCommands(); i++)
      {
         InverseKinematicsCommand<?> commandToCopy = other.getCommand(i);

         switch (commandToCopy.getCommandType())
         {
         case JOINTSPACE:
            copyJointspaceVelocityCommand((JointspaceVelocityCommand) commandToCopy);
            break;
         case MOMENTUM:
            copyMomentumCommand((MomentumCommand) commandToCopy);
            break;
         case TASKSPACE:
            copySpatialVelocityCommand((SpatialVelocityCommand) commandToCopy);
         default:
            throw new RuntimeException("The command type: " + commandToCopy.getCommandType() + " is not handled.");
         }
      }
   }

   private void copyJointspaceVelocityCommand(JointspaceVelocityCommand commandToCopy)
   {
      JointspaceVelocityCommand localCommand = jointspaceVelocityCommands.add();
      localCommand.set(commandToCopy);
      inverseKinematicsCommandList.addCommand(localCommand);
   }

   private void copyMomentumCommand(MomentumCommand commandToCopy)
   {
      MomentumCommand localCommand = momentumCommands.add();
      localCommand.set(commandToCopy);
      inverseKinematicsCommandList.addCommand(localCommand);
   }

   private void copySpatialVelocityCommand(SpatialVelocityCommand commandToCopy)
   {
      SpatialVelocityCommand localCommand = spatialVelocityCommands.add();
      localCommand.set(commandToCopy);
      inverseKinematicsCommandList.addCommand(localCommand);
   }

   public InverseKinematicsCommandList getInverseKinematicsCommandList()
   {
      return inverseKinematicsCommandList;
   }

   private void clear()
   {
      inverseKinematicsCommandList.clear();
      jointspaceVelocityCommands.clear();
      spatialVelocityCommands.clear();
      momentumCommands.clear();
   }
}
