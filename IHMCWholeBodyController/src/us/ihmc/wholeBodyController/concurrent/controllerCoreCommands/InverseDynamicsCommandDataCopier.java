package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class InverseDynamicsCommandDataCopier
{
   private static final int INITIAL_CAPACITY = 20;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   private final RecyclingArrayList<ExternalWrenchCommand> externalWrenchCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, ExternalWrenchCommand.class);
   private final RecyclingArrayList<JointspaceAccelerationCommand> jointspaceAccelerationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, JointspaceAccelerationCommand.class);
   private final RecyclingArrayList<MomentumRateCommand> momentumRateCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, MomentumRateCommand.class);
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, PlaneContactStateCommand.class);
   private final RecyclingArrayList<CenterOfPressureCommand> centerOfPressureCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, CenterOfPressureCommand.class);
   private final RecyclingArrayList<PointAccelerationCommand> pointAccelerationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, PointAccelerationCommand.class);
   private final RecyclingArrayList<SpatialAccelerationCommand> spatialAccelerationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, SpatialAccelerationCommand.class);
   private final RecyclingArrayList<JointAccelerationIntegrationCommand> jointAccelerationIntegrationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, JointAccelerationIntegrationCommand.class);

   public InverseDynamicsCommandDataCopier()
   {
      clear();
   }

   public void retrieveRigidBodiesFromName(Map<String, RigidBody> nameToRigidBodyMap)
   {
      for (int i = 0; i < externalWrenchCommands.size(); i++)
      {
         ExternalWrenchCommand command = externalWrenchCommands.get(i);
         command.setRigidBody(nameToRigidBodyMap.get(command.getRigidBodyName()));
      }

      for (int i = 0; i < planeContactStateCommands.size(); i++)
      {
         PlaneContactStateCommand command = planeContactStateCommands.get(i);
         command.setContactingRigidBody(nameToRigidBodyMap.get(command.getContactingRigidBodyName()));
      }

      for (int i = 0; i < centerOfPressureCommands.size(); i++)
      {
         CenterOfPressureCommand command = centerOfPressureCommands.get(i);
         command.setContactingRigidBody(nameToRigidBodyMap.get(command.getContactingRigidBodyName()));
      }

      for (int i = 0; i < pointAccelerationCommands.size(); i++)
      {
         PointAccelerationCommand command = pointAccelerationCommands.get(i);
         RigidBody base = nameToRigidBodyMap.get(command.getBaseName());
         RigidBody endEffector = nameToRigidBodyMap.get(command.getEndEffectorName());
         command.set(base, endEffector);
      }

      for (int i = 0; i < spatialAccelerationCommands.size(); i++)
      {
         SpatialAccelerationCommand command = spatialAccelerationCommands.get(i);
         RigidBody base = nameToRigidBodyMap.get(command.getBaseName());
         RigidBody endEffector = nameToRigidBodyMap.get(command.getEndEffectorName());
         command.set(base, endEffector);
      }
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < jointspaceAccelerationCommands.size(); i++)
      {
         JointspaceAccelerationCommand command = jointspaceAccelerationCommands.get(i);
         command.retrieveJointsFromName(nameToJointMap);
      }
   }

   public void copyFromOther(InverseDynamicsCommandList other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfCommands(); i++)
      {
         InverseDynamicsCommand<?> commandToCopy = other.getCommand(i);

         switch (commandToCopy.getCommandType())
         {
         case EXTERNAL_WRENCH:
            copyExternalWrenchCommand((ExternalWrenchCommand) commandToCopy);
            break;
         case JOINTSPACE:
            copyJointspaceAccelerationCommand((JointspaceAccelerationCommand) commandToCopy);
            break;
         case MOMENTUM:
            copyMomentumRateCommand((MomentumRateCommand) commandToCopy);
            break;
         case PLANE_CONTACT_STATE:
            copyPlaneContactStateCommand((PlaneContactStateCommand) commandToCopy);
            break;
         case CENTER_OF_PRESSURE:
            copyCenterOfPressureCommand((CenterOfPressureCommand) commandToCopy);
            break;
         case POINT:
            copyPointAcclerationCommand((PointAccelerationCommand) commandToCopy);
            break;
         case TASKSPACE:
            copySpatialAccelerationCommand((SpatialAccelerationCommand) commandToCopy);
            break;
         case JOINT_ACCELERATION_INTEGRATION:
            copyJointAccelerationIntegrationCommand((JointAccelerationIntegrationCommand) commandToCopy);
            break;
         case COMMAND_LIST:
            copyFromOther((InverseDynamicsCommandList) commandToCopy);
            break;
         default:
            throw new RuntimeException("The command type: " + commandToCopy.getCommandType() + " is not handled.");
         }
      }
   }

   private void clear()
   {
      inverseDynamicsCommandList.clear();
      externalWrenchCommands.clear();
      jointspaceAccelerationCommands.clear();
      momentumRateCommands.clear();
      planeContactStateCommands.clear();
      centerOfPressureCommands.clear();
      pointAccelerationCommands.clear();
      spatialAccelerationCommands.clear();
      jointAccelerationIntegrationCommands.clear();
   }

   private void copyExternalWrenchCommand(ExternalWrenchCommand commandToCopy)
   {
      ExternalWrenchCommand localCommand = externalWrenchCommands.add();
      localCommand.set(commandToCopy);
      inverseDynamicsCommandList.addCommand(localCommand);
   }

   private void copyJointspaceAccelerationCommand(JointspaceAccelerationCommand commandToCopy)
   {
      JointspaceAccelerationCommand localCommand = jointspaceAccelerationCommands.add();
      localCommand.set(commandToCopy);
      inverseDynamicsCommandList.addCommand(localCommand);
   }

   private void copyMomentumRateCommand(MomentumRateCommand commandToCopy)
   {
      MomentumRateCommand localCommand = momentumRateCommands.add();
      localCommand.set(commandToCopy);
      inverseDynamicsCommandList.addCommand(localCommand);
   }

   private void copyPlaneContactStateCommand(PlaneContactStateCommand commandToCopy)
   {
      PlaneContactStateCommand localCommand = planeContactStateCommands.add();
      localCommand.set(commandToCopy);
      inverseDynamicsCommandList.addCommand(localCommand);
   }

   private void copyCenterOfPressureCommand(CenterOfPressureCommand commandToCopy)
   {
      CenterOfPressureCommand localCommand = centerOfPressureCommands.add();
      localCommand.set(commandToCopy);
      inverseDynamicsCommandList.addCommand(localCommand);
   }

   private void copyPointAcclerationCommand(PointAccelerationCommand commandToCopy)
   {
      PointAccelerationCommand localCommand = pointAccelerationCommands.add();
      localCommand.set(commandToCopy);
      inverseDynamicsCommandList.addCommand(localCommand);
   }

   private void copySpatialAccelerationCommand(SpatialAccelerationCommand commandToCopy)
   {
      SpatialAccelerationCommand localCommand = spatialAccelerationCommands.add();
      localCommand.set(commandToCopy);
      inverseDynamicsCommandList.addCommand(localCommand);
   }

   private void copyJointAccelerationIntegrationCommand(JointAccelerationIntegrationCommand commandToCopy)
   {
      JointAccelerationIntegrationCommand localCommand = jointAccelerationIntegrationCommands.add();
      localCommand.set(commandToCopy);
      inverseDynamicsCommandList.addCommand(localCommand);
   }

   public InverseDynamicsCommandList getInverseDynamicsCommandList()
   {
      return inverseDynamicsCommandList;
   }
}
