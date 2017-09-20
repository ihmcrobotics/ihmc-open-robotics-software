package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class ConstrainedWholeBodyPlanningToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FullHumanoidRobotModel fullRobotModelToUseForConversion;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingInverseDynamicsJoint rootJoint;
   private final OneDoFJoint[] oneDoFJoints;
   private final int jointsHashCode;
   
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   
   private double trajectoryTime = 0.0;
   
   private double firstTrajectoryPointTime = 6.0;
   
   private ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;
   
   public ConstrainedWholeBodyPlanningToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      this.fullRobotModelToUseForConversion = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModelToUseForConversion.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModelToUseForConversion);
      jointsHashCode = (int) NameBasedHashCodeTools.computeArrayHashCode(oneDoFJoints);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModelToUseForConversion);
   }
   
   public WholeBodyTrajectoryMessage getWholebodyTrajectoryMessage()
   {
      return wholebodyTrajectoryMessage;
   }

   public void setMessageToCreate(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.wholebodyTrajectoryMessage = wholebodyTrajectoryMessage;
   }
   
   public void setConstrainedEndEffectorTrajectory(ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory)
   {
      this.constrainedEndEffectorTrajectory = constrainedEndEffectorTrajectory;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   public double getFirstTrajectoryPointTime()
   {
      return firstTrajectoryPointTime;
   }

   public void setFirstTrajectoryPointTime(double firstTrajectoryPointTime)
   {
      this.firstTrajectoryPointTime = firstTrajectoryPointTime;
   }

   public void updateFullRobotModel(ConstrainedWholeBodyPlanningToolboxOutputStatus solution)
   {
//      if (jointsHashCode != solution.jointNameHash)
//         throw new RuntimeException("Hashes are different.");
//
//      for (int i = 0; i < oneDoFJoints.length; i++)
//      {
//         float q = solution.getJointAngles()[i];
//         OneDoFJoint joint = oneDoFJoints[i];
//         joint.setQ(q);
//      }
//      
//      Vector3D32 translation = solution.getPelvisTranslation();
//      rootJoint.setPosition(translation.getX(), translation.getY(), translation.getZ());
//      Quaternion32 orientation = solution.getPelvisOrientation();
//      rootJoint.setRotation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
//      fullRobotModelToUseForConversion.updateFrames();
   }
}
