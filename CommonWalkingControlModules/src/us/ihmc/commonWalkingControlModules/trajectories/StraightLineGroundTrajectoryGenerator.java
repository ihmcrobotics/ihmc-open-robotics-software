package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class StraightLineGroundTrajectoryGenerator implements GroundTrajectoryGenerator
{

   private final YoVariableRegistry registry;
   
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   
   
   private final FramePoint startPoint;
   private final FramePoint endPoint;
   
   
   private final DoubleYoVariable absoluteYOffsetForViaPoints; 
   
   public StraightLineGroundTrajectoryGenerator(String name, CommonHumanoidReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);      
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      
      startPoint = new FramePoint(ReferenceFrame.getWorldFrame());
      endPoint = new FramePoint(ReferenceFrame.getWorldFrame());
      new FrameVector(ReferenceFrame.getWorldFrame());
      
      parentRegistry.addChild(registry);
      
      absoluteYOffsetForViaPoints = new DoubleYoVariable("absoluteYOffsetForViaPoints", registry);
      absoluteYOffsetForViaPoints.set(0.02);
   }

   public void getViaPoints(YoFramePoint[] viaPointsToPack, RobotSide swingSide, double tStart, FramePoint startPointIn, double tEnd, FramePoint endPointIn, double[] tOfViaPoints, double heightOfViaPoints[])
   {
      
      ReferenceFrame groundPlaneFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
      
      
      startPoint.setIncludingFrame(startPointIn);
      startPoint.changeFrame(groundPlaneFrame);
      
      endPoint.setIncludingFrame(endPointIn);
      endPoint.changeFrame(groundPlaneFrame);
      
      double deltaX = endPoint.getX() - startPoint.getX();
      double deltaY = endPoint.getY() - startPoint.getY();
      
      double tScale = 1.0 / (tEnd - tStart);
      
      
      for(int i = 0; i < tOfViaPoints.length; i++)
      {
         FramePoint viaPoint = new FramePoint(groundPlaneFrame);
         
         double yOffset = 0.0;
         
         yOffset = swingSide.negateIfRightSide(absoluteYOffsetForViaPoints.getDoubleValue());
         
         
         viaPoint.set(startPoint.getX() + tScale*(tOfViaPoints[i]-tStart)*deltaX, 
                      startPoint.getY() + tScale*(tOfViaPoints[i]-tStart)*deltaY + yOffset, 
                      heightOfViaPoints[i]);
         
         viaPoint.changeFrame(viaPointsToPack[i].getReferenceFrame());
         viaPointsToPack[i].set(viaPoint);
         
         
      }
      
   }

}
