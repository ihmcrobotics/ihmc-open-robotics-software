package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class ConstrainedWholeBodyPlanningRequestPacket extends Packet<ConstrainedWholeBodyPlanningRequestPacket>
{
   public int numberOfExpanding;

   public int numberOfFindInitialGuess;

//   public OneDoFJoint[] initialOneDoFJoints; // TODO look at RobotConfigurationData
//
//   public Vector3D initialTranslationOfRootJoint;
//   public Quaternion initialRotationOfRootJoint;
          
   public AtlasKinematicsConfiguration initialConfiguration;

   /*
    * ConstrainedEndEffectorTrajectory cannot be imported in
    * IHMCHumanoidRobotics.
    */
   //public ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   public ConstrainedWholeBodyPlanningRequestPacket()
   {

   }

   public void setInitialRobotConfigration(FullHumanoidRobotModel fullRobotModel)
   {
//      initialOneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
//      initialTranslationOfRootJoint = new Vector3D(fullRobotModel.getRootJoint().getTranslationForReading());
//      initialRotationOfRootJoint = new Quaternion(fullRobotModel.getRootJoint().getRotationForReading());
      
      initialConfiguration = new AtlasKinematicsConfiguration();
      
      initialConfiguration.putJointConfiguration(FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel));
      initialConfiguration.putRootTranslation(fullRobotModel.getRootJoint().getTranslationForReading());      
      initialConfiguration.putRootOrientation(fullRobotModel.getRootJoint().getRotationForReading());
   }

   public void setNumberOfExpanding(int value)
   {
      this.numberOfExpanding = value;
   }

   public void setNumberOfFindInitialGuess(int value)
   {
      this.numberOfFindInitialGuess = value;
   }

   @Override
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningRequestPacket other, double epsilon)
   {
      return true;
   }

}
