package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelCleaningBehavior
{
   private SolarPanel solarPanel;
   private WholeBodyInverseKinematicsBehavior ik;
   
   private SolarPanelStraightTrajectory linearTrajectory;
   
   public SolarPanelCleaningBehavior(SolarPanel panel, WholeBodyInverseKinematicsBehavior ik)
   {
      this.solarPanel = panel;
      this.ik = ik;
   }
   
   public void setIKwithGoalPose(SolarPanelCleaningPose cleaningPose, double motionTime)
   {  
      FramePose desiredHandPoseR = new FramePose();
      desiredHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      
      desiredHandPoseR.setPose(cleaningPose.getLocation(), cleaningPose.getRotation());
      ik.setTrajectoryTime(motionTime);
      ik.setDesiredHandPose(RobotSide.RIGHT, desiredHandPoseR);
   }
   
   public HandTrajectoryMessage getHandTrajectoryMessage(double motionTime)
   {  
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, BaseForControl.WORLD, linearTrajectory.getPoses().size());

      Vector3D desiredLinearVelocity = new Vector3D();
      Vector3D desiredAngularVelocity = new Vector3D();
      
      for(int i =0;i<linearTrajectory.getPoses().size();i++)
      {
         SolarPanelCleaningPose solarPanelCleaningPose = linearTrajectory.getPoses().get(i);
         
         PrintTools.info(""+i+" "+solarPanelCleaningPose.getLocation().getX()+" "+solarPanelCleaningPose.getLocation().getY()+" "+solarPanelCleaningPose.getLocation().getZ());
         handTrajectoryMessage.setTrajectoryPoint(i, motionTime, solarPanelCleaningPose.getLocation(), solarPanelCleaningPose.getRotation(), desiredLinearVelocity, desiredAngularVelocity);
      }
      
      
      return handTrajectoryMessage;
   }
   
   public void generateMotion()
   {
      SolarPanelCleaningPose cleaningPoseOne = new SolarPanelCleaningPose(new Point3D(0.5,  -0.25,  1.2), 0.0, -Math.PI/0.25, 0.0);
            
      SolarPanelCleaningPose cleaningPoseTwo = new SolarPanelCleaningPose(new Point3D(0.5,  -0.55,  1.2), 0.0, -Math.PI/0.25, 0.0);
            
      linearTrajectory = new SolarPanelStraightTrajectory(cleaningPoseOne, cleaningPoseTwo, 10);
   }
   
   public SolarPanelStraightTrajectory getTrajectory()
   {
      return linearTrajectory;
   }
}
