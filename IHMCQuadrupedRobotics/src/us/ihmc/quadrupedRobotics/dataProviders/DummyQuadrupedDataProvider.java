package us.ihmc.quadrupedRobotics.dataProviders;

import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.trajectory.QuadrupedSwingTrajectoryGenerator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.CartesianTrajectoryBasedFootSwitch;
import us.ihmc.robotics.sensors.FootSwitchProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class DummyQuadrupedDataProvider implements QuadrupedDataProvider
{
   private final QuadrantDependentList<FootSwitchProvider> footSwitchProviders = new QuadrantDependentList<>();
   private final QuadrantDependentList<QuadrupedSwingTrajectoryGenerator> swingTrajectoryGenerators = new QuadrantDependentList<>();

   public DummyQuadrupedDataProvider(double dt, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedSwingTrajectoryGenerator trajectoryGenerator = new QuadrupedSwingTrajectoryGenerator(referenceFrames, robotQuadrant, registry, yoGraphicsListRegistry, dt);
         swingTrajectoryGenerators.set(robotQuadrant, trajectoryGenerator);
         footSwitchProviders.set(robotQuadrant, new CartesianTrajectoryBasedFootSwitch(robotQuadrant.getCamelCaseNameForStartOfExpression() + "FootSwitch",
               trajectoryGenerator.getTrajectoryGenerator(), registry));
      }
   }

   @Override
   public VectorProvider getDesiredVelocityProvider()
   {
      return null;
   }

   @Override
   public DoubleProvider getDesiredYawRateProvider()
   {
      return null;
   }

   @Override
   public FootSwitchProvider getFootSwitchProvider(RobotQuadrant robotQuadrant)
   {
      return footSwitchProviders.get(robotQuadrant);
   }

   @Override
   public QuadrupedSwingTrajectoryGenerator getSwingTrajectoryGenerator(RobotQuadrant robotQuadrant)
   {
      return swingTrajectoryGenerators.get(robotQuadrant);
   }
}
