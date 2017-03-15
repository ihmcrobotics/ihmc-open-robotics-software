package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelCleaningBehavior
{
   private SolarPanel solarPanel;
   private WholeBodyInverseKinematicsBehavior ik;
   
   private SolarPanelStraightTrajectory solarPanelTrajectory;
   
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
   
   public void generateMotion()
   {
      SolarPanelCleaningPose cleaningPoseOne = new SolarPanelCleaningPose(new Point3D(0.5,  -0.25,  1.2), 0.0, -Math.PI/0.25, 0.0);
            
      SolarPanelCleaningPose cleaningPoseTwo = new SolarPanelCleaningPose(new Point3D(0.5,  -0.55,  1.2), 0.0, -Math.PI/0.25, 0.0);
            
      solarPanelTrajectory = new SolarPanelStraightTrajectory(cleaningPoseOne, cleaningPoseTwo, 10);
   }
   
   public SolarPanelStraightTrajectory getTrajectory()
   {
      return solarPanelTrajectory;
   }
}
