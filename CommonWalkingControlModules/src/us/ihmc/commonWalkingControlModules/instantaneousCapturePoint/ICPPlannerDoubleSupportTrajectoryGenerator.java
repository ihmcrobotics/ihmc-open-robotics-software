package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;

public class ICPPlannerDoubleSupportTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final VelocityConstrainedPositionTrajectoryGenerator doubleSupportCapturePointTrajectory;

   private final FramePoint initialPositionInSpecificFrame = new FramePoint();
   private final FrameVector initialVelocityInSpecificFrame = new FrameVector();
   private final FramePoint finalPositionInSpecificFrame = new FramePoint();
   private final FrameVector finalVelocityInSpecificFrame = new FrameVector();

   public ICPPlannerDoubleSupportTrajectoryGenerator(String namePrefix, int numberOfCoefficients, YoVariableRegistry registry)
   {
      doubleSupportCapturePointTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix + "DoubleSupportTrajectory", numberOfCoefficients, worldFrame, registry);
   }

   public void setTrajectoryTime(double duration)
   {
      doubleSupportCapturePointTrajectory.setTrajectoryTime(duration);
   }

   public void setInitialConditions(FramePoint initialPosition, FrameVector initialVelocity)
   {
      initialPositionInSpecificFrame.setIncludingFrame(initialPosition);
      initialVelocityInSpecificFrame.setIncludingFrame(initialVelocity);
   }

   public void setFinalConditions(FramePoint finalPosition, FrameVector finalVelocity)
   {
      finalPositionInSpecificFrame.setIncludingFrame(finalPosition);
      finalVelocityInSpecificFrame.setIncludingFrame(finalVelocity);
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

   @Override
   public void packVelocity(FrameVector velocityToPack)
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
