package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;

public class ICPPlannerTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final VelocityConstrainedPositionTrajectoryGenerator doubleSupportCapturePointTrajectory;

   private final FramePoint initialPositionInSpecificFrame = new FramePoint();
   private final FrameVector initialVelocityInSpecificFrame = new FrameVector();
   private final FramePoint finalPositionInSpecificFrame = new FramePoint();
   private final FrameVector finalVelocityInSpecificFrame = new FrameVector();

   public ICPPlannerTrajectoryGenerator(String namePrefix, ReferenceFrame trajectoryFrame, YoVariableRegistry registry)
   {
      doubleSupportCapturePointTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix, 4, trajectoryFrame, registry);
   }

   public void setTrajectoryTime(double duration)
   {
      doubleSupportCapturePointTrajectory.setTrajectoryTime(duration);
   }

   public void setInitialConditions(YoFramePoint initialPosition, YoFrameVector initialVelocity, ReferenceFrame attachedFrame)
   {
      initialPosition.getFrameTupleIncludingFrame(initialPositionInSpecificFrame);
      initialVelocity.getFrameTupleIncludingFrame(initialVelocityInSpecificFrame);
      initialPositionInSpecificFrame.changeFrame(attachedFrame);
      initialVelocityInSpecificFrame.changeFrame(attachedFrame);
   }
   
   public void setFinalConditions(YoFramePoint finalPosition, YoFrameVector finalVelocity, ReferenceFrame attachedFrame)
   {
      finalPosition.getFrameTupleIncludingFrame(finalPositionInSpecificFrame);
      finalVelocity.getFrameTupleIncludingFrame(finalVelocityInSpecificFrame);
      finalPositionInSpecificFrame.changeFrame(attachedFrame);
      finalVelocityInSpecificFrame.changeFrame(attachedFrame);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void compute(double time)
   {
      doubleSupportCapturePointTrajectory.setInitialConditions(initialPositionInSpecificFrame, initialVelocityInSpecificFrame);
      doubleSupportCapturePointTrajectory.setFinalConditions(finalPositionInSpecificFrame, finalVelocityInSpecificFrame);
      doubleSupportCapturePointTrajectory.initialize();
      doubleSupportCapturePointTrajectory.compute(time);
   }

   @Override
   public boolean isDone()
   {
      return doubleSupportCapturePointTrajectory.isDone();
   }

   @Override
   public void get(FramePoint positionToPack)
   {
      doubleSupportCapturePointTrajectory.get(positionToPack);
   }

   public void get(YoFramePoint positionToPack)
   {
      doubleSupportCapturePointTrajectory.get(positionToPack);
   }

   @Override
   public void packVelocity(FrameVector velocityToPack)
   {
      doubleSupportCapturePointTrajectory.packVelocity(velocityToPack);
   }

   public void packVelocity(YoFrameVector velocityToPack)
   {
      doubleSupportCapturePointTrajectory.packVelocity(velocityToPack);
   }

   @Override
   public void packAcceleration(FrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.packAcceleration(accelerationToPack);
   }

   @Override
   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.packLinearData(positionToPack, velocityToPack, accelerationToPack);
   }

   public void packLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.packLinearData(positionToPack, velocityToPack, accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
      doubleSupportCapturePointTrajectory.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      doubleSupportCapturePointTrajectory.hideVisualization();
   }
}
