package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.PositionProvider;


public class ConstantPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final YoFramePoint position;
   private final DoubleYoVariable finalTime;
   private final DoubleYoVariable time;
   private final PositionProvider positionProvider;

   public ConstantPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, PositionProvider positionProvider, double finalTime,
           YoVariableRegistry parentRegistry)
   {
      MathTools.checkIntervalContains(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.positionProvider = positionProvider;
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.position = new YoFramePoint("position", referenceFrame, registry);
      this.finalTime = new DoubleYoVariable("finalTime", registry);
      this.time = new DoubleYoVariable("time", registry);
      this.finalTime.set(finalTime);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      time.set(0.0);
      FramePoint positionToPack = new FramePoint();
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

   public void getPosition(FramePoint positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getVelocity(FrameVector velocityToPack)
   {
      velocityToPack.setToZero(position.getReferenceFrame());
   }

   public void getAcceleration(FrameVector accelerationToPack)
   {
      accelerationToPack.setToZero(position.getReferenceFrame());
   }

   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void showVisualization()
   {
      // TODO Auto-generated method stub
   }

   public void hideVisualization()
   {
      // TODO Auto-generated method stub
   }
}
