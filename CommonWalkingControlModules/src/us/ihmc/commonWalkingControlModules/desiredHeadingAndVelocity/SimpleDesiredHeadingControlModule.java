package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.RateBasedDesiredHeadingControlModule.DesiredHeadingFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;

public class SimpleDesiredHeadingControlModule implements DesiredHeadingControlModule
{
   private SimpleDesiredHeadingControlModuleVisualizer simpleDesiredHeadingControlModuleVisualizer;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable desiredHeadingFinal = new DoubleYoVariable("desiredHeadingFinal",
         "Yaw of the desired heading frame with respect to the world.", registry);
   private final DoubleYoVariable desiredHeading = new DoubleYoVariable("desiredHeading", registry);
   private final DoubleYoVariable maxHeadingDot = new DoubleYoVariable("maxHeadingDot", "In units of rad/sec", registry);

   private final DesiredHeadingFrame desiredHeadingFrame = new DesiredHeadingFrame();
   private final DesiredHeadingFrame predictedHeadingFrame = new DesiredHeadingFrame();

   private final double controlDT;

   public SimpleDesiredHeadingControlModule(double desiredHeadingfinal, double controlDT, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.controlDT = controlDT;

      maxHeadingDot.set(0.1);

      desiredHeadingFinal.set(desiredHeadingfinal);
      desiredHeading.set(desiredHeadingFinal.getDoubleValue()); // The final is the first one according to the initial setup of the robot

      updateDesiredHeadingFrame();
   }

   public void setMaxHeadingDot(double maxHeadingDot)
   {
      this.maxHeadingDot.set(maxHeadingDot);
   }

   public double getMaxHeadingDot()
   {
      return maxHeadingDot.getDoubleValue();
   }

   public void initializeVisualizer(ProcessedSensorsInterface processedSensors, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (simpleDesiredHeadingControlModuleVisualizer != null)
      {
         throw new RuntimeException("Already setupVisualizer");
      }

      simpleDesiredHeadingControlModuleVisualizer = new SimpleDesiredHeadingControlModuleVisualizer(processedSensors, registry, yoGraphicsListRegistry);
   }

   @Override
   public void updateDesiredHeadingFrame()
   {
      updateDesiredHeading();
      desiredHeadingFrame.setHeadingAngleAndUpdate(desiredHeading.getDoubleValue());

      if (simpleDesiredHeadingControlModuleVisualizer != null)
      {
         simpleDesiredHeadingControlModuleVisualizer.updateDesiredHeading(desiredHeading.getDoubleValue(), desiredHeadingFinal.getDoubleValue());
      }
   }

   @Override
   public double getFinalHeadingTargetAngle()
   {
      return desiredHeadingFinal.getDoubleValue();
   }

   @Override
   public FrameVector2d getFinalHeadingTarget()
   {
      FrameVector2d finalHeading = new FrameVector2d(ReferenceFrame.getWorldFrame(), Math.cos(desiredHeadingFinal.getDoubleValue()),
            Math.sin(desiredHeadingFinal.getDoubleValue()));

      return finalHeading;
   }

   @Override
   public ReferenceFrame getDesiredHeadingFrame()
   {
      return desiredHeadingFrame;
   }

   @Override
   public ReferenceFrame getPredictedHeadingFrame(double timeFromNow)
   {
      predictedHeadingFrame.setHeadingAngleAndUpdate(predictDesiredHeading(timeFromNow));
      return predictedHeadingFrame;
   }

   @Override
   public void setFinalHeadingTargetAngle(double finalHeadingTargetAngle)
   {
      desiredHeadingFinal.set(finalHeadingTargetAngle);
   }

   @Override
   public void setFinalHeadingTarget(FrameVector2d finalHeadingTarget)
   {
      finalHeadingTarget.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setFinalHeadingTargetAngle(Math.atan2(finalHeadingTarget.getY(), finalHeadingTarget.getX()));
   }

   @Override
   public double getDesiredHeadingAngle()
   {
      return desiredHeading.getDoubleValue();
   }

   @Override
   public void getDesiredHeading(FrameVector2d desiredHeadingToPack, double timeFromNow)
   {
      double heading = predictDesiredHeading(timeFromNow);
      desiredHeadingToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), Math.cos(heading), Math.sin(heading));
   }

   @Override
   public void resetHeadingAngle(double newHeading)
   {
      desiredHeading.set(newHeading);
      desiredHeadingFinal.set(newHeading);
   }

   private void updateDesiredHeading()
   {
      double error = desiredHeadingFinal.getDoubleValue() - desiredHeading.getDoubleValue();
      double maximumChangePerTick = maxHeadingDot.getDoubleValue() * controlDT;

      double deltaHeading = MathTools.clamp(error, -maximumChangePerTick, maximumChangePerTick);

      desiredHeading.set(desiredHeading.getDoubleValue() + deltaHeading);
   }

   private double predictDesiredHeading(double timeFromNow)
   {
      double error = desiredHeadingFinal.getDoubleValue() - desiredHeading.getDoubleValue();
      double maximumChange = maxHeadingDot.getDoubleValue() * timeFromNow;

      double deltaHeading = MathTools.clamp(error, -maximumChange, maximumChange);

      return desiredHeading.getDoubleValue() + deltaHeading;
   }
}
