package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public class ConstantPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final YoFramePoint position;
   private final YoDouble finalTime;
   private final YoDouble time;
   private final PositionProvider positionProvider;

   public ConstantPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, PositionProvider positionProvider, double finalTime,
           YoVariableRegistry parentRegistry)
   {
      MathTools.checkIntervalContains(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.positionProvider = positionProvider;
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.position = new YoFramePoint("position", referenceFrame, registry);
      this.finalTime = new YoDouble("finalTime", registry);
      this.time = new YoDouble("time", registry);
      this.finalTime.set(finalTime);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      time.set(0.0);
      FramePoint3D positionToPack = new FramePoint3D();
      positionProvider.getPosition(positionToPack);
      positionToPack.changeFrame(position.getReferenceFrame());
      position.set(positionToPack);
   }

   public void compute(double time)
   {
      this.time.set(time);
   }

   public boolean isDone()
   {
      return time.getDoubleValue() > finalTime.getDoubleValue();
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setToZero(position.getReferenceFrame());
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setToZero(position.getReferenceFrame());
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void showVisualization()
   {
   }

   public void hideVisualization()
   {
   }
}
