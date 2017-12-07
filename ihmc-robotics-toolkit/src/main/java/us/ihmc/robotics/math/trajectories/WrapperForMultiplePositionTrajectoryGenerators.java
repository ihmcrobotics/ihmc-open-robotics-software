package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class WrapperForMultiplePositionTrajectoryGenerators implements PositionTrajectoryGenerator
{
   private final String namePostfix = getClass().getSimpleName();

   private final YoBoolean replanPositionTrajectory;

   private final YoInteger positionTrajectoryGeneratorIndex;
   private final ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators;
   private final YoDouble timeIntoStep;

   public WrapperForMultiplePositionTrajectoryGenerators(ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators, String namePrefix,
         YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + namePostfix);
      parentRegistry.addChild(registry);

      this.positionTrajectoryGenerators = positionTrajectoryGenerators;
      positionTrajectoryGeneratorIndex = new YoInteger(namePrefix + "PositionTrajectoryGeneratorIndex", registry);
      timeIntoStep = new YoDouble(namePrefix + "TimeIntoStep", registry);

      this.replanPositionTrajectory = new YoBoolean(namePrefix + "ReplanPositionTrajectory", registry);
      this.replanPositionTrajectory.set(false);
   }

   public void initialize()
   {
      positionTrajectoryGeneratorIndex.set(0);
      for (int i = 0; i < positionTrajectoryGenerators.size(); i++)
      {
         positionTrajectoryGenerators.get(i).initialize();
      }
   }

   public void compute(double time)
   {
      if (positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue()).isDone()
            && (positionTrajectoryGeneratorIndex.getIntegerValue() < positionTrajectoryGenerators.size() - 1))
      {
         positionTrajectoryGeneratorIndex.increment();
      }

      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.compute(time);
      timeIntoStep.set(time);
   }

   public boolean isDone()
   {
      boolean currentTrajectoryIsLast = positionTrajectoryGeneratorIndex.getIntegerValue() == positionTrajectoryGenerators.size() - 1;
      boolean currentTrajectoryIsDone = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue()).isDone();
      return currentTrajectoryIsLast && currentTrajectoryIsDone;
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.getPosition(positionToPack);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.getVelocity(velocityToPack);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.getAcceleration(accelerationToPack);
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.getLinearData(positionToPack, velocityToPack, accelerationToPack);
   }

   public void showVisualization()
   {
   }

   public void hideVisualization()
   {
   }
}
