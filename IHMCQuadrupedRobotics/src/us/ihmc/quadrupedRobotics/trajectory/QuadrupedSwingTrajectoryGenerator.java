package us.ihmc.quadrupedRobotics.trajectory;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.Finishable;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedSwingTrajectoryGenerator
{
   private final double DEFAULT_SWING_TIME = 0.3;
   private final boolean USE_NEW_SWING_GENERATOR = true;

   private final double dt;
   
   private final ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator cartesianTrajectoryGenerator;
   private final ParabolicCartesianTrajectoryGenerator parabolicCartesianTrajectoryGenerator;

   private final RobotQuadrant robotQuadrant;

   private final YoVariableRegistry registry;
   private final YoVariableDoubleProvider swingTimeDoubleProvider;
   
   private final FramePoint desiredEndEffectorPosition = new FramePoint();
   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector finalDesiredVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector zeroVector = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);

   private BagOfBalls bagOfBalls;
   private int ballCounter = 0;


   public QuadrupedSwingTrajectoryGenerator(RobotQuadrant robotQuadrant, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, double dt)
   {
      this.dt = dt;
      this.robotQuadrant = robotQuadrant;
      String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
      registry = new YoVariableRegistry(prefix + "MiniBeastSwingTrajectoryGenerator");
      swingTimeDoubleProvider = new YoVariableDoubleProvider(prefix + "swingTime", registry);
      swingTimeDoubleProvider.set(DEFAULT_SWING_TIME);
      
      cartesianTrajectoryGenerator = new ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator("swingLegTraj", ReferenceFrame.getWorldFrame(), registry);


      DoubleProvider stepTimeProvider = new DoubleProvider()
      {
         @Override public double getValue()
         {
            return swingTimeDoubleProvider.getValue();
         }
      };

      parabolicCartesianTrajectoryGenerator = new ParabolicCartesianTrajectoryGenerator("swingLegTrajCart", ReferenceFrame.getWorldFrame(), stepTimeProvider, Double.POSITIVE_INFINITY, registry);

      parentRegistry.addChild(registry);

      bagOfBalls = new BagOfBalls(50, 0.01, prefix + "SwingTrajectoryBoB", registry, yoGraphicsListRegistry);
   }
   
   public void initializeSwing(double swingTime, FramePoint swingInitial, double swingHeight, FramePoint swingTarget, FrameVector desiredFinalVelocity)
   {
      bagOfBalls.reset();
      
      swingTimeDoubleProvider.set(swingTime);
      initialPosition.setIncludingFrame(swingInitial);
      finalDesiredVelocity.set(desiredFinalVelocity);

      cartesianTrajectoryGenerator.setTrajectoryParameters(swingTime, initialPosition, swingHeight, swingTarget, finalDesiredVelocity);
      cartesianTrajectoryGenerator.initialize();

      //New traj generator
      parabolicCartesianTrajectoryGenerator.updateGroundClearance(swingHeight);
      parabolicCartesianTrajectoryGenerator.initialize(initialPosition, zeroVector, zeroVector, swingTarget, finalDesiredVelocity);
   }
   
   public void computeSwing(FramePoint framePointToPack)
   {
      if (USE_NEW_SWING_GENERATOR)
      {
         parabolicCartesianTrajectoryGenerator.computeNextTick(framePointToPack, dt);
         updateBagOfBalls(framePointToPack);
      }
      else
      {
         cartesianTrajectoryGenerator.compute(dt);
         cartesianTrajectoryGenerator.get(desiredEndEffectorPosition);
         framePointToPack.setIncludingFrame(desiredEndEffectorPosition);
         updateBagOfBalls(desiredEndEffectorPosition);
      }
   }

   private void updateBagOfBalls(FramePoint desiredEndEffectorPosition)
   {
      if(ballCounter % 10000 == 0)
      {
         desiredEndEffectorPosition.changeFrame(ReferenceFrame.getWorldFrame());
         bagOfBalls.setBall(desiredEndEffectorPosition, YoAppearance.Azure());
      }
      ballCounter++;
   }

   public boolean isDone()
   {
      if (USE_NEW_SWING_GENERATOR)
      {
         return parabolicCartesianTrajectoryGenerator.isDone();
      }
      else
         return cartesianTrajectoryGenerator.isDone();
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public Finishable getTrajectoryGenerator()
   {
      return cartesianTrajectoryGenerator;
   }

}
