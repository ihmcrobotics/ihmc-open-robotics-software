package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class ConstrainedWholeBodyPlanningRequestPacket extends Packet<ConstrainedWholeBodyPlanningRequestPacket>
{       
   public int numberOfExpanding;
      
   public OneDoFJoint[] initialOneDoFJoints;
   
   public Vector3D initialTranslationOfRootJoint;
   public Quaternion initialRotationOfRootJoint;
   
   /*
    * ConstrainedEndEffectorTrajectory cannot be imported in IHMCHumanoidRobotics.
    */
   //public ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   public ConstrainedWholeBodyPlanningRequestPacket()
   {

   }
   
   public void setInitialRobotConfigration(FullHumanoidRobotModel fullRobotModel)
   {
      initialOneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);      
      initialTranslationOfRootJoint = new Vector3D(fullRobotModel.getRootJoint().getTranslationForReading());
      initialRotationOfRootJoint = new Quaternion(fullRobotModel.getRootJoint().getRotationForReading());
   }
   
   public void setNumberOfExpanding(int value)
   {
      this.numberOfExpanding = value;
   }

   @Override
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningRequestPacket other, double epsilon)
   {
      return true;
   }

}
