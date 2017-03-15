package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPPlannerTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final VelocityConstrainedPositionTrajectoryGenerator doubleSupportCapturePointTrajectory;

   private final FramePoint initialPositionInSpecificFrame = new FramePoint();
   private final FrameVector initialVelocityInSpecificFrame = new FrameVector();
   private final FramePoint finalPositionInSpecificFrame = new FramePoint();
   private final FrameVector finalVelocityInSpecificFrame = new FrameVector();

   public ICPPlannerTrajectoryGenerator(String namePrefix, ReferenceFrame trajectoryFrame, YoVariableRegistry registry)
   {
      doubleSupportCapturePointTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix, trajectoryFrame, registry);
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
   public void getPosition(FramePoint positionToPack)
   {
      doubleSupportCapturePointTrajectory.getPosition(positionToPack);
   }

   public void get(YoFramePoint positionToPack)
   {
      doubleSupportCapturePointTrajectory.get(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      doubleSupportCapturePointTrajectory.getVelocity(velocityToPack);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      doubleSupportCapturePointTrajectory.getVelocity(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.getAcceleration(accelerationToPack);
   }

   public void getAcceleration(YoFrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.getAcceleration(accelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.getLinearData(positionToPack, velocityToPack, accelerationToPack);
   }

   public void getLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.getLinearData(positionToPack, velocityToPack, accelerationToPack);
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
