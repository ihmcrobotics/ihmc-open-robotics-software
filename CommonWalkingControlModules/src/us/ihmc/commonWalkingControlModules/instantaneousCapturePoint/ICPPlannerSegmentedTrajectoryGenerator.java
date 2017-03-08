package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPPlannerSegmentedTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;

   private final DoubleYoVariable omega0;

   private final DoubleYoVariable maximumSplineDuration;
   private final DoubleYoVariable minimumSplineDuration;
   private final DoubleYoVariable minimumTimeToSpendOnExitCMP;
   private final DoubleYoVariable totalTrajectoryTime;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable timeSpentOnInitialCMP;
   private final DoubleYoVariable timeSpentOnFinalCMP;
   private final DoubleYoVariable progressionInPercent;
   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;

   private final YoFramePoint yoStartOfSplineICP;
   private final YoFramePoint yoEndOfSplineICP;

   private final IntegerYoVariable currentSegment;

   private ReferenceFrame trajectoryFrame;
   private ReferenceFrame initialFrame;
   private ReferenceFrame finalFrame;

   private final FramePoint initialCornerPointInitialFrame = new FramePoint();
   private final FramePoint finalCornerPointInitialFrame = new FramePoint();
   private final FramePoint initialCMPInitialFrame = new FramePoint();
   private final FramePoint finalCMPInitialFrame = new FramePoint();
   private final FramePoint initialICPInitialFrame = new FramePoint();
   private final FramePoint finalICPInitialFrame = new FramePoint();

   private final FramePoint startOfSplineICPInitialFrame = new FramePoint();
   private final FrameVector startOfSplineICPVelocityInitialFrame = new FrameVector();
   private final FramePoint endOfSplineICPInitialFrame = new FramePoint();
   private final FrameVector endOfSplineICPVelocityInitialFrame = new FrameVector();

   private final FramePoint initialCornerPointFinalFrame = new FramePoint();
   private final FramePoint finalCornerPointFinalFrame = new FramePoint();
   private final FramePoint initialCMPFinalFrame = new FramePoint();
   private final FramePoint finalCMPFinalFrame = new FramePoint();
   private final FramePoint initialICPFinalFrame = new FramePoint();
   private final FramePoint finalICPFinalFrame = new FramePoint();

   private final FramePoint startOfSplineICPFinalFrame = new FramePoint();
   private final FrameVector startOfSplineICPVelocityFinalFrame = new FrameVector();
   private final FramePoint endOfSplineICPFinalFrame = new FramePoint();
   private final FrameVector endOfSplineICPVelocityFinalFrame = new FrameVector();

   private final FramePoint startOfSplineICP = new FramePoint();
   private final FrameVector startOfSplineICPVelocity = new FrameVector();
   private final FramePoint endOfSplineICP = new FramePoint();
   private final FrameVector endOfSplineICPVelocity = new FrameVector();

   private final FramePoint desiredICPInitialFrame = new FramePoint();
   private final FrameVector desiredICPVelocityInitialFrame = new FrameVector();

   private final FramePoint desiredICPFinalFrame = new FramePoint();
   private final FrameVector desiredICPVelocityFinalFrame = new FrameVector();

   private final FramePoint desiredICPOutput = new FramePoint();
   private final FrameVector desiredICPVelocityOutput = new FrameVector();

   private final VelocityConstrainedPositionTrajectoryGenerator spline;

   public ICPPlannerSegmentedTrajectoryGenerator(String namePrefix, ReferenceFrame trajectoryFrame, DoubleYoVariable omega0,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.trajectoryFrame = trajectoryFrame;
      initialFrame = trajectoryFrame;
      finalFrame = trajectoryFrame;

      this.omega0 = omega0;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      maximumSplineDuration = new DoubleYoVariable(namePrefix + "MaximumSplineDuration", registry);
      minimumSplineDuration = new DoubleYoVariable(namePrefix + "MinimumSplineDuration", registry);
      minimumSplineDuration.set(0.1);
      minimumTimeToSpendOnExitCMP = new DoubleYoVariable(namePrefix + "MinimumTimeToSpendOnExitCMP", registry);
      totalTrajectoryTime = new DoubleYoVariable(namePrefix + "TotalTrajectoryTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "CurrentTime", registry);
      timeSpentOnInitialCMP = new DoubleYoVariable(namePrefix + "TimeSpentOnInitialCMP", registry);
      timeSpentOnFinalCMP = new DoubleYoVariable(namePrefix + "TimeSpentOnFinalCMP", registry);
      progressionInPercent = new DoubleYoVariable(namePrefix + "ProgressionInPercent", registry);
      startOfSplineTime = new DoubleYoVariable(namePrefix + "StartOfSplineTime", registry);
      endOfSplineTime = new DoubleYoVariable(namePrefix + "EndOfSplineTime", registry);

      currentSegment = new IntegerYoVariable(namePrefix + "CurrentSegment", registry);

      spline = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix, trajectoryFrame, registry);

      yoStartOfSplineICP = new YoFramePoint(namePrefix + "InitialICPSpline", trajectoryFrame, registry);
      yoEndOfSplineICP = new YoFramePoint(namePrefix + "FinalICPSpline", trajectoryFrame, registry);

      parentRegistry.addChild(registry);
   }

   public void createVisualizers(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      YoGraphicPosition startOfSplineICPViz = new YoGraphicPosition("singleSupportInitialSplineICP", yoStartOfSplineICP, 0.004, YoAppearance.Black(),
            GraphicType.SOLID_BALL);
      yoGraphicsList.add(startOfSplineICPViz);
      artifactList.add(startOfSplineICPViz.createArtifact());

      YoGraphicPosition endOfSplineICPViz = new YoGraphicPosition("singleSupportFinalSplineICP", yoEndOfSplineICP, 0.004, YoAppearance.Black(),
            GraphicType.BALL);
      yoGraphicsList.add(endOfSplineICPViz);
      artifactList.add(endOfSplineICPViz.createArtifact());

   }

   public void setMaximumSplineDuration(double maximumSplineDuration)
   {
      this.maximumSplineDuration.set(maximumSplineDuration);
   }

   public void setMinimumTimeToSpendOnExitCMP(double duration)
   {
      minimumTimeToSpendOnExitCMP.set(duration);
   }

   public void setReferenceFrames(ReferenceFrame initialFrame, ReferenceFrame finalFrame)
   {
      this.initialFrame = initialFrame;
      this.finalFrame = finalFrame;
   }

   public void setTrajectoryTime(double timeSpentOnInitialCMP, double timeSpentOnFinalCMP)
   {
      this.timeSpentOnInitialCMP.set(timeSpentOnInitialCMP);
      this.timeSpentOnFinalCMP.set(timeSpentOnFinalCMP);
      totalTrajectoryTime.set(timeSpentOnInitialCMP + timeSpentOnFinalCMP);
   }

   public void setCornerPoints(YoFramePoint initialCornerPoint, YoFramePoint finalCornerPoint)
   {
      initialCornerPoint.getFrameTupleIncludingFrame(initialCornerPointInitialFrame);
      initialCornerPoint.getFrameTupleIncludingFrame(initialCornerPointFinalFrame);

      finalCornerPoint.getFrameTupleIncludingFrame(finalCornerPointInitialFrame);
      finalCornerPoint.getFrameTupleIncludingFrame(finalCornerPointFinalFrame);
   }

   public void setReferenceCMPs(YoFramePoint initialCMP, YoFramePoint finalCMP)
   {
      initialCMP.getFrameTupleIncludingFrame(initialCMPInitialFrame);
      initialCMP.getFrameTupleIncludingFrame(initialCMPFinalFrame);

      finalCMP.getFrameTupleIncludingFrame(finalCMPInitialFrame);
      finalCMP.getFrameTupleIncludingFrame(finalCMPFinalFrame);
   }

   public void setBoundaryICP(YoFramePoint initialICP, YoFramePoint finalICP)
   {
      initialICP.getFrameTupleIncludingFrame(initialICPInitialFrame);
      initialICP.getFrameTupleIncludingFrame(initialICPFinalFrame);

      finalICP.getFrameTupleIncludingFrame(finalICPInitialFrame);
      finalICP.getFrameTupleIncludingFrame(finalICPFinalFrame);
   }

   @Override
   public void initialize()
   {
      initialCornerPointInitialFrame.changeFrame(initialFrame);
      finalCornerPointInitialFrame.changeFrame(initialFrame);
      initialCMPInitialFrame.changeFrame(initialFrame);
      finalCMPInitialFrame.changeFrame(initialFrame);
      initialICPInitialFrame.changeFrame(initialFrame);
      finalICPInitialFrame.changeFrame(initialFrame);

      initialCornerPointFinalFrame.changeFrame(finalFrame);
      finalCornerPointFinalFrame.changeFrame(finalFrame);
      initialCMPFinalFrame.changeFrame(finalFrame);
      finalCMPFinalFrame.changeFrame(finalFrame);
      initialICPFinalFrame.changeFrame(finalFrame);
      finalICPFinalFrame.changeFrame(finalFrame);

      double alpha = 0.50;
      double minTimeOnExitCMP = minimumTimeToSpendOnExitCMP.getDoubleValue();
      minTimeOnExitCMP = Math.min(minTimeOnExitCMP, timeSpentOnFinalCMP.getDoubleValue() - alpha * minimumSplineDuration.getDoubleValue());

      double startOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() - alpha * maximumSplineDuration.getDoubleValue();
      startOfSplineTime = Math.max(startOfSplineTime, 0.0);
      this.startOfSplineTime.set(startOfSplineTime);

      double endOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() + (1.0 - alpha) * maximumSplineDuration.getDoubleValue();
      endOfSplineTime = Math.min(endOfSplineTime, totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP);
      if (endOfSplineTime > totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP)
      {
         endOfSplineTime = totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP;
         startOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() - (endOfSplineTime - timeSpentOnInitialCMP.getDoubleValue());
      }
      this.startOfSplineTime.set(startOfSplineTime);
      this.endOfSplineTime.set(endOfSplineTime);

      spline.setTrajectoryTime(endOfSplineTime - startOfSplineTime);

      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), startOfSplineTime, initialICPInitialFrame, initialCMPInitialFrame,
            startOfSplineICPInitialFrame);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), startOfSplineTime, initialICPInitialFrame, initialCMPInitialFrame,
            startOfSplineICPVelocityInitialFrame);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), endOfSplineTime - timeSpentOnInitialCMP.getDoubleValue(),
            finalCornerPointInitialFrame, finalCMPInitialFrame, endOfSplineICPInitialFrame);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), endOfSplineTime - timeSpentOnInitialCMP.getDoubleValue(),
            finalCornerPointInitialFrame, finalCMPInitialFrame, endOfSplineICPVelocityInitialFrame);

      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), startOfSplineTime, initialICPFinalFrame, initialCMPFinalFrame,
            startOfSplineICPFinalFrame);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), startOfSplineTime, initialICPFinalFrame, initialCMPFinalFrame,
            startOfSplineICPVelocityFinalFrame);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), endOfSplineTime - timeSpentOnInitialCMP.getDoubleValue(),
            finalCornerPointFinalFrame, finalCMPFinalFrame, endOfSplineICPFinalFrame);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), endOfSplineTime - timeSpentOnInitialCMP.getDoubleValue(),
            finalCornerPointFinalFrame, finalCMPFinalFrame, endOfSplineICPVelocityFinalFrame);
   }

   @Override
   public void compute(double time)
   {
      time = MathTools.clamp(time, 0.0, totalTrajectoryTime.getDoubleValue());
      progressionInPercent.set(time / totalTrajectoryTime.getDoubleValue());

      updateSplineBoundaries();

      if (time <= startOfSplineTime.getDoubleValue())
      {
         currentSegment.set(1);
         computeFirstSegment(time);
      }
      else if (time >= endOfSplineTime.getDoubleValue())
      {
         currentSegment.set(3);
         computeThirdSegment(time - endOfSplineTime.getDoubleValue());
      }
      else
      {
         currentSegment.set(2);
         computeSecondSegment(time - startOfSplineTime.getDoubleValue());
      }
   }

   private void computeFirstSegment(double timeInFirstSegment)
   {
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), timeInFirstSegment, initialICPInitialFrame, initialCMPInitialFrame,
            desiredICPInitialFrame);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), timeInFirstSegment, initialICPFinalFrame, initialCMPFinalFrame,
            desiredICPFinalFrame);

      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), timeInFirstSegment, initialICPInitialFrame, initialCMPInitialFrame,
            desiredICPVelocityInitialFrame);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), timeInFirstSegment, initialICPFinalFrame, initialCMPFinalFrame,
            desiredICPVelocityFinalFrame);

      interpolatePointFromInitialToFinalFrame(desiredICPOutput, desiredICPInitialFrame, desiredICPFinalFrame, progressionInPercent.getDoubleValue());
      interpolateVectorFromInitialToFinalFrame(desiredICPVelocityOutput, desiredICPVelocityInitialFrame, desiredICPVelocityFinalFrame,
            progressionInPercent.getDoubleValue());
   }

   private void computeSecondSegment(double timeInSecondSegment)
   {
      spline.setInitialConditions(startOfSplineICP, startOfSplineICPVelocity);
      spline.setFinalConditions(endOfSplineICP, endOfSplineICPVelocity);
      spline.initialize();
      spline.compute(timeInSecondSegment);

      spline.getPosition(desiredICPOutput);
      spline.getVelocity(desiredICPVelocityOutput);
   }

   private void computeThirdSegment(double timeInThirdSegment)
   {
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), timeInThirdSegment, endOfSplineICPInitialFrame, finalCMPInitialFrame,
            desiredICPInitialFrame);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), timeInThirdSegment, endOfSplineICPFinalFrame, finalCMPFinalFrame,
            desiredICPFinalFrame);

      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), timeInThirdSegment, endOfSplineICPInitialFrame, finalCMPInitialFrame,
            desiredICPVelocityInitialFrame);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), timeInThirdSegment, endOfSplineICPFinalFrame, finalCMPFinalFrame,
            desiredICPVelocityFinalFrame);

      interpolatePointFromInitialToFinalFrame(desiredICPOutput, desiredICPInitialFrame, desiredICPFinalFrame, progressionInPercent.getDoubleValue());
      interpolateVectorFromInitialToFinalFrame(desiredICPVelocityOutput, desiredICPVelocityInitialFrame, desiredICPVelocityFinalFrame,
            progressionInPercent.getDoubleValue());
   }

   private void updateSplineBoundaries()
   {
      double progressionInPercent = startOfSplineTime.getDoubleValue() / totalTrajectoryTime.getDoubleValue();
      interpolatePointFromInitialToFinalFrame(startOfSplineICP, startOfSplineICPInitialFrame, startOfSplineICPFinalFrame, progressionInPercent);
      interpolateVectorFromInitialToFinalFrame(startOfSplineICPVelocity, startOfSplineICPVelocityInitialFrame, startOfSplineICPVelocityFinalFrame,
            progressionInPercent);

      progressionInPercent = endOfSplineTime.getDoubleValue() / totalTrajectoryTime.getDoubleValue();
      interpolatePointFromInitialToFinalFrame(endOfSplineICP, endOfSplineICPInitialFrame, endOfSplineICPFinalFrame, progressionInPercent);
      interpolateVectorFromInitialToFinalFrame(endOfSplineICPVelocity, endOfSplineICPVelocityInitialFrame, endOfSplineICPVelocityFinalFrame,
            progressionInPercent);

      yoStartOfSplineICP.set(startOfSplineICP);
      yoEndOfSplineICP.set(endOfSplineICP);
   }

   private final FramePoint pointA = new FramePoint();
   private final FramePoint pointB = new FramePoint();

   private void interpolatePointFromInitialToFinalFrame(FramePoint pointTrajectoryFrameToPack, FramePoint pointInitialFrame, FramePoint pointFinalFrame,
         double percentOfFinal)
   {
      pointA.setIncludingFrame(pointInitialFrame);
      pointB.setIncludingFrame(pointFinalFrame);
      pointA.changeFrame(trajectoryFrame);
      pointB.changeFrame(trajectoryFrame);
      pointTrajectoryFrameToPack.setToZero(trajectoryFrame);
      pointTrajectoryFrameToPack.interpolate(pointA, pointB, percentOfFinal);
   }

   private final FrameVector vectorA = new FrameVector();
   private final FrameVector vectorB = new FrameVector();

   private void interpolateVectorFromInitialToFinalFrame(FrameVector vectorTrajectoryFrameToPack, FrameVector vectorInitialFrame, FrameVector vectorFinalFrame,
         double percentOfFinal)
   {
      vectorA.setIncludingFrame(vectorInitialFrame);
      vectorB.setIncludingFrame(vectorFinalFrame);
      vectorA.changeFrame(trajectoryFrame);
      vectorB.changeFrame(trajectoryFrame);
      vectorTrajectoryFrameToPack.setToZero(trajectoryFrame);
      vectorTrajectoryFrameToPack.interpolate(vectorA, vectorB, percentOfFinal);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= totalTrajectoryTime.getDoubleValue();
   }

   @Override
   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.setIncludingFrame(desiredICPOutput);
   }

   public boolean isOnExitCMP()
   {
      return progressionInPercent.getDoubleValue() * totalTrajectoryTime.getDoubleValue() > endOfSplineTime.getDoubleValue();
   }

   public void get(YoFramePoint positionToPack)
   {
      positionToPack.set(desiredICPOutput);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      velocityToPack.setIncludingFrame(desiredICPVelocityOutput);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(desiredICPVelocityOutput);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      accelerationToPack.setToZero(trajectoryFrame);
   }

   public void getAcceleration(YoFrameVector accelerationToPack)
   {
      accelerationToPack.setToZero();
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
      yoStartOfSplineICP.setToNaN();
      yoEndOfSplineICP.setToNaN();
   }
}
