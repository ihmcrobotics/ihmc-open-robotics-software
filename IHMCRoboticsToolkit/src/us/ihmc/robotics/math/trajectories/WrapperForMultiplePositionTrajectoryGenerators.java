package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;


public class WrapperForMultiplePositionTrajectoryGenerators implements PositionTrajectoryGenerator
{
   private final String namePostfix = getClass().getSimpleName();

   private final BooleanYoVariable replanPositionTrajectory;

   private final IntegerYoVariable positionTrajectoryGeneratorIndex;
   private final ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators;
   private final DoubleYoVariable timeIntoStep;

   public WrapperForMultiplePositionTrajectoryGenerators(ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators, String namePrefix,
         YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + namePostfix);
      parentRegistry.addChild(registry);

      this.positionTrajectoryGenerators = positionTrajectoryGenerators;
      positionTrajectoryGeneratorIndex = new IntegerYoVariable(namePrefix + "PositionTrajectoryGeneratorIndex", registry);
      timeIntoStep = new DoubleYoVariable(namePrefix + "TimeIntoStep", registry);

      this.replanPositionTrajectory = new BooleanYoVariable(namePrefix + "ReplanPositionTrajectory", registry);
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

   public void get(FramePoint positionToPack)
   {
      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.get(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.packVelocity(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.packAcceleration(accelerationToPack);
   }

   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      PositionTrajectoryGenerator currentGenerator = positionTrajectoryGenerators.get(positionTrajectoryGeneratorIndex.getIntegerValue());
      currentGenerator.packLinearData(positionToPack, velocityToPack, accelerationToPack);
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
