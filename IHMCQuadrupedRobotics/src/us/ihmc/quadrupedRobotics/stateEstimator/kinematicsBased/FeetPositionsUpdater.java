package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class FeetPositionsUpdater
{
   private final QuadrantDependentList<FramePoint> feetPositions = new QuadrantDependentList<FramePoint>();
   private QuadrantDependentList<OneDoFJoint[]> legJointsTrees = new QuadrantDependentList<OneDoFJoint[]>();
   private RobotQuadrant swingLeg;
   
   public FeetPositionsUpdater(QuadrantDependentList<OneDoFJoint[]> legJointsTrees)
   {
      this.legJointsTrees = legJointsTrees;
      
   }
   
   public void initialize()
   {
      System.out.println("initialize the FeetPositionUpdater");
   }
   
   public void updateFeetPositions()
   {
      
   }
   
   public void setSwingLeg(RobotQuadrant swingLeg)
   {
      this.swingLeg = swingLeg;
   }
   
}
