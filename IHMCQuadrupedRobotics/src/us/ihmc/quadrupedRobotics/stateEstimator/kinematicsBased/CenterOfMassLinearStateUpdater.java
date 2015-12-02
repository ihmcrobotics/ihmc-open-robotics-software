package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class CenterOfMassLinearStateUpdater
{
   
   private final QuadrantDependentList<OneDoFJoint[]> legJointsTrees = new QuadrantDependentList<OneDoFJoint[]>();
   private final QuadrantDependentList<FramePoint> feetPositions = new QuadrantDependentList<FramePoint>();
   private RobotQuadrant swingLeg;
   
   public CenterOfMassLinearStateUpdater()
   {
      
   }
   
   
   public void initialize()
   {
      System.out.println("init the CenterOfMassLinearStateUpdater");
   }
   
   public void updateCenterOfMassLinearState()
   {
      
   }
   
   public void setSwingLeg(RobotQuadrant swingLeg)
   {
      this.swingLeg = swingLeg;
   }
   
   //trusted feet
   //update CoMPose
}
