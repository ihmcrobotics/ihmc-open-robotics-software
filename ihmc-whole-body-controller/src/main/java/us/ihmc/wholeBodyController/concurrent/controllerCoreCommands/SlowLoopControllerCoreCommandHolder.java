package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandInterface;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SlowLoopControllerCoreCommandHolder
{
   private final Map<String, RigidBody> fastLoopRigidBodyMap = new HashMap<>();
   private final Map<String, OneDoFJoint> fastLoopJointMap = new HashMap<>();

   private final ControllerCoreCommandDataCopier intermediateCommandDataCopier = new ControllerCoreCommandDataCopier();
   private final ControllerCoreCommandDataCopier fastLoopCommandDataCopier = new ControllerCoreCommandDataCopier();
   
   public SlowLoopControllerCoreCommandHolder(FullHumanoidRobotModel fastLoopFullRobotModel)
   {
      setupRigidBodyMap(fastLoopFullRobotModel, fastLoopRigidBodyMap);
      for (OneDoFJoint joint : fastLoopFullRobotModel.getOneDoFJoints())
         fastLoopJointMap.put(joint.getName(), joint);
   }

   public void writeSlowLoopData(ControllerCoreCommand commandToWrite)
   {
      intermediateCommandDataCopier.copyDataFrom(commandToWrite);
   }

   public void readFastLoopData()
   {
      fastLoopCommandDataCopier.copyDataFrom(intermediateCommandDataCopier);
      fastLoopCommandDataCopier.retrieveRigidBodiesFromName(fastLoopRigidBodyMap);
      fastLoopCommandDataCopier.retrieveJointsFromName(fastLoopJointMap);
   }

   public ControllerCoreCommandInterface getFastLoopControllerCoreCommand()
   {
      return fastLoopCommandDataCopier;
   }

   // No need to put all of the rigid bodies (there is a lot especially when having hands).
   private static void setupRigidBodyMap(FullHumanoidRobotModel fullRobotModel, Map<String, RigidBody> rigidBodyMapToPack)
   {
      addRigidBodyToMap(fullRobotModel.getElevator(), rigidBodyMapToPack);
      addRigidBodyToMap(fullRobotModel.getHead(), rigidBodyMapToPack);
      addRigidBodyToMap(fullRobotModel.getChest(), rigidBodyMapToPack);
      addRigidBodyToMap(fullRobotModel.getPelvis(), rigidBodyMapToPack);

      for (RobotSide robotSide : RobotSide.values)
      {
         addRigidBodyToMap(fullRobotModel.getHand(robotSide), rigidBodyMapToPack);
         addRigidBodyToMap(fullRobotModel.getFoot(robotSide), rigidBodyMapToPack);
      }
   }

   public static void addRigidBodyToMap(RigidBody foot, Map<String, RigidBody> rigidBodyMapToPack)
   {
      rigidBodyMapToPack.put(foot.getName(), foot);
   }

   public static class Builder implements us.ihmc.concurrent.Builder<SlowLoopControllerCoreCommandHolder>
   {
      private final FullHumanoidRobotModel fastLoopFullRobotModel;

      public Builder(FullHumanoidRobotModel fastLoopFullRobotModel)
      {
         this.fastLoopFullRobotModel = fastLoopFullRobotModel;
      }

      @Override
      public SlowLoopControllerCoreCommandHolder newInstance()
      {
         return new SlowLoopControllerCoreCommandHolder(fastLoopFullRobotModel);
      }
   }
}
