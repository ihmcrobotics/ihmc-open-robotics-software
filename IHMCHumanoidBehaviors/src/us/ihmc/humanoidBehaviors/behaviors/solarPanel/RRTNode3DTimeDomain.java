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
      
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(nodeValidityTester.getDesiredFullRobotModel());
      referenceFrames.updateFrames();
      ReferenceFrame rootFrame = referenceFrames.getMidFootZUpGroundFrame();
      
      // Hand
      FramePoint desiredHandFramePoint = new FramePoint(rootFrame, new Point3D(0.6, -0.4, 1.0));
      Quaternion desriedOrientation = new Quaternion();
      desriedOrientation.appendYawRotation(Math.PI*0.5);
      desriedOrientation.appendPitchRotation(Math.PI*0.5);
      FrameOrientation desiredHandFrameOrientation = new FrameOrientation(rootFrame, desriedOrientation);
      
      FramePose desiredHandFramePose = new FramePose(desiredHandFramePoint, desiredHandFrameOrientation);
      desiredHandFramePose.changeFrame(referenceFrames.getWorldFrame());
      
      nodeValidityTester.setDesiredHandPose(RobotSide.RIGHT, desiredHandFramePose);
      FramePose desiredHandFramePose2 = new FramePose();
      desiredHandFramePose2.setToNaN();
      nodeValidityTester.setDesiredHandPose(RobotSide.LEFT, desiredHandFramePose2);
      
      nodeValidityTester.putTrajectoryMessages();
      
      for(int i=0;i<30;i++)
      {
         nodeValidityTester.updateInternal();
         ThreadTools.sleep(10);
      }
      
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
