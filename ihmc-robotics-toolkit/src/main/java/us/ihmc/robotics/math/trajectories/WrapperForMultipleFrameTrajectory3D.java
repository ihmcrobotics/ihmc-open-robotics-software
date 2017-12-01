package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public class WrapperForMultipleFrameTrajectory3D
{
   private final String namePostfix = getClass().getSimpleName();

   private final YoInteger positionTrajectoryGeneratorIndex;
   private final ArrayList<FrameTrajectory3D> frameTrajectories;
   private final YoDouble timeIntoStep;

   public WrapperForMultipleFrameTrajectory3D(ArrayList<FrameTrajectory3D> frameTrajectories, String namePrefix, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + namePostfix);
      parentRegistry.addChild(registry);

      this.frameTrajectories = frameTrajectories;
      positionTrajectoryGeneratorIndex = new YoInteger(namePrefix + "PositionTrajectoryGeneratorIndex", registry);
      timeIntoStep = new YoDouble(namePrefix + "TimeIntoStep", registry);
   }

   public void initialize()
   {
      positionTrajectoryGeneratorIndex.set(0);
   }

   public void compute(double time)
   {
      if (!frameTrajectories.get(positionTrajectoryGeneratorIndex.getIntegerValue()).timeIntervalContains(time)
            && (positionTrajectoryGeneratorIndex.getIntegerValue() < frameTrajectories.size() - 1))
      {
         positionTrajectoryGeneratorIndex.increment();
      }

      FrameTrajectory3D currentGenerator = frameTrajectories.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.compute(time);
      timeIntoStep.set(time);
   }

   public boolean isDone()
   {
      if (!frameTrajectories.get(positionTrajectoryGeneratorIndex.getIntegerValue()).timeIntervalContains(timeIntoStep.getDoubleValue()))
      {
         if (positionTrajectoryGeneratorIndex.getIntegerValue() >= frameTrajectories.size())
            return true;
         else
            return false;
      }

      return false;
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      FrameTrajectory3D currentGenerator = frameTrajectories.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      positionToPack.set(currentGenerator.getPosition());
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      FrameTrajectory3D currentGenerator = frameTrajectories.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      velocityToPack.set(currentGenerator.getVelocity());
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      FrameTrajectory3D currentGenerator = frameTrajectories.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      accelerationToPack.set(currentGenerator.getAcceleration());
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      FrameTrajectory3D currentGenerator = frameTrajectories.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      positionToPack.set(currentGenerator.getFramePosition());
      velocityToPack.set(currentGenerator.getFrameVelocity());
      accelerationToPack.set(currentGenerator.getFrameAcceleration());
   }
}
