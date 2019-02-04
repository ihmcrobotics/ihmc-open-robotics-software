package us.ihmc.avatar.rigidbodies;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class RigidBodyHashTest
{
   private static final int maximumNumberOfTimes = 30;

   @ContinuousIntegrationTest(estimatedDuration = 2.0)
   @Test(timeout = 30000)
   public void testSignificantRigidBodiesHashCode()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         printNameIDAndHashCode(hand);
         
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         printNameIDAndHashCode(foot);
      }

      RigidBodyBasics chest = fullRobotModel.getChest();
      printNameIDAndHashCode(chest);

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      printNameIDAndHashCode(pelvis);

      RigidBodyBasics head = fullRobotModel.getHead();
      printNameIDAndHashCode(head);
      assertTrue(true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 3.0)
   @Test(timeout = 50000)
   public void testAllRigidBodiesHashCode()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         for (int i = 0; i < maximumNumberOfTimes; i++)
         {
            if (printNameIDAndHashCode(hand) == getElevatorHashCode())
               break;

            hand = hand.getParentJoint().getPredecessor();
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);

         for (int i = 0; i < maximumNumberOfTimes; i++)
         {
            if (printNameIDAndHashCode(foot) == getElevatorHashCode())
               break;

            foot = foot.getParentJoint().getPredecessor();
         }
      }
      
      RigidBodyBasics head = fullRobotModel.getHead();

      for (int i = 0; i < maximumNumberOfTimes; i++)
      {
         if (printNameIDAndHashCode(head) == getElevatorHashCode())
            break;

         head = head.getParentJoint().getPredecessor();
      }
      assertTrue(true);
   }

   private int printNameIDAndHashCode(RigidBodyBasics rigidBody)
   {
      String rigidBodyName = rigidBody.getName();
      String rigidBodyNameID = rigidBody.getNameId();
      int hashCode = rigidBodyName.hashCode();
      int hashCodeID = rigidBodyNameID.hashCode();
      LogTools.info(rigidBodyName + " string ID is " + rigidBodyNameID);
      LogTools.info(rigidBodyName + " hash code is " + hashCodeID);
      return hashCode;
   }

   private int getElevatorHashCode()
   {
      DRCRobotModel robotModel = getRobotModel();

      return robotModel.createFullRobotModel().getElevator().getNameId().hashCode();
   }

   public abstract DRCRobotModel getRobotModel();
}
