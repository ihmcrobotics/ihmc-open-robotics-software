package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public class DRCRobotWalkingControllerParameters implements WalkingControllerParameters
{
   public SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame()
   {
      SideDependentList<Transform3D> handPoseWithRespectToChestFrame = new SideDependentList<Transform3D>();
      for(RobotSide robotSide : RobotSide.values)
      {
         double[] mat = { 0, robotSide.negateIfLeftSide(-0.7), 0.71, 0.25,
               robotSide.negateIfLeftSide(1), 0, 0, robotSide.negateIfLeftSide(-0.30),
               0, robotSide.negateIfLeftSide(0.71), 0.7, -0.44,
               0.0, 0.0, 0.0, 1.0 };
         
         final Transform3D transform3d = new Transform3D(mat);
         
         handPoseWithRespectToChestFrame.put(robotSide, transform3d);
      }
      
      return handPoseWithRespectToChestFrame;
   }
   
   public double getDesiredCoMHeight()
   {
      return 0.88;
   }

   public boolean doStrictPelvisControl()
   {
      return true;
   }

   public String[] neckJointsToUseForHeadOrientationControl()
   {
      return new String[] { "neck_ay" };
   }
   
   public boolean checkOrbitalCondition() 
   {
      return false;
   }
}
