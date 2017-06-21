package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.WheneverWholeBodyValidityTester;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.thread.ThreadTools;

public class RRTNode3DTimeDomain extends RRTNode
{
   public static WheneverWholeBodyValidityTester nodeValidityTester;
   public static SolarPanelPath cleaningPath;
   
   public RRTNode3DTimeDomain()
   {
      super(4);
   }

   public RRTNode3DTimeDomain(double timeK, double pelvisHeight, double chestYaw, double chestPitch)
   {
      super(4);
      super.setNodeData(0, timeK);
      super.setNodeData(1, pelvisHeight);
      super.setNodeData(2, chestYaw);
      super.setNodeData(3, chestPitch);
   }  
   
   @Override
   public boolean isValidNode()
   {
      PrintTools.info("isvalid START");
      
      nodeValidityTester.initialize();      
      nodeValidityTester.holdCurrentTrajectoryMessages();
      
      // Hand
      Quaternion desiredHandOrientation = new Quaternion();
      desiredHandOrientation.appendPitchRotation(Math.PI*30/180);
      nodeValidityTester.setDesiredHandPose(RobotSide.RIGHT, new Pose(new Point3D(0.6, -0.4, 1.0), desiredHandOrientation));
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendPitchRotation(Math.PI*10/180);
      nodeValidityTester.setDesiredChestOrientation(desiredChestOrientation);
            
      nodeValidityTester.setDesiredPelvisHeight(0.75);
      
      nodeValidityTester.putTrajectoryMessages();
      
      
      
      PrintTools.info("isvalid END");
      return true;
   }

   @Override
   public RRTNode createNode()
   {
      return new RRTNode3DTimeDomain();
   }

   @Override
   public void setRandomNodeData()
   {
      // TODO Auto-generated method stub
      
   }
}
