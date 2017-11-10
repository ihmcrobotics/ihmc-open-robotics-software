package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class WholeBodyTrajectoryToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FullHumanoidRobotModel fullRobotModelToUseForConversion;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingInverseDynamicsJoint rootJoint;
   private final OneDoFJoint[] oneDoFJoints;
   private final int jointsHashCode;

   private WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage;

   private double trajectoryTime = 0.0;

   private double firstTrajectoryPointTime = 6.0;

   private ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   public WholeBodyTrajectoryToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      this.fullRobotModelToUseForConversion = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModelToUseForConversion.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModelToUseForConversion);
      jointsHashCode = (int) NameBasedHashCodeTools.computeArrayHashCode(oneDoFJoints);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModelToUseForConversion);
   }

   public WholeBodyTrajectoryMessage getWholebodyTrajectoryMessage()
   {
      return wholeBodyTrajectoryMessage;
   }

   public void setMessageToCreate(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.wholeBodyTrajectoryMessage = wholebodyTrajectoryMessage;
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

   public void updateFullRobotModel(WholeBodyTrajectoryToolboxOutputStatus solution)
   {

   }
   
   public KinematicsToolboxOutputStatus getRobotConfiguration(WholeBodyTrajectoryToolboxOutputStatus solution, double time)
   {
      double minimumGap = Double.MAX_VALUE;
      int nodeIndex = 0;
      for(int i=0;i<solution.getTrajectoryTimes().length;i++)
      {
         double gap = Math.abs(time - solution.getTrajectoryTimes()[i]);
         if(gap < minimumGap)
         {
            minimumGap = gap;
            nodeIndex = i;
         }
      }
      
      return solution.getRobotConfigurations()[nodeIndex];
   }
}
