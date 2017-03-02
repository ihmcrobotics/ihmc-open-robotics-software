package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.alphaToAlpha.AlphaToAlphaFunction;
import us.ihmc.robotics.alphaToAlpha.MultipleSegmentConstantSlope;
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

   private final DoubleYoVariable timeInStep, alphaTimeInStep, alphaIn, alphaOut;
   private final DoubleYoVariable alphaSlopeAtStart;
   private final AlphaToAlphaFunction alphaToAlphaFunction;
   private BagOfBalls bagOfBalls;
   private int ballCounter = 0;


   public QuadrupedSwingTrajectoryGenerator(RobotQuadrant robotQuadrant, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, double dt)
   {
      this.dt = dt;
      this.robotQuadrant = robotQuadrant;
      String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
      registry = new YoVariableRegistry(prefix + "QuadrupedSwingTrajectoryGenerator");
      swingTimeDoubleProvider = new YoVariableDoubleProvider(prefix + "swingTime", registry);
      swingTimeDoubleProvider.set(DEFAULT_SWING_TIME);
      
      cartesianTrajectoryGenerator = new ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator(prefix + "swingLegTraj", ReferenceFrame.getWorldFrame(), registry);
      timeInStep = new DoubleYoVariable(prefix + "TimeInStep", registry);
      alphaTimeInStep = new DoubleYoVariable(prefix + "AlphaTimeInStep", registry);
      alphaIn = new DoubleYoVariable(prefix + "AlphaIn", registry);
      alphaOut = new DoubleYoVariable(prefix + "AlphaOut", registry);
      alphaSlopeAtStart = new DoubleYoVariable(prefix + "AlphaSlopeAtStart", registry);
      alphaSlopeAtStart.set(1.5);

      //stretchedSlowAtEndAlphaToAlphaFunction = new StretchedSlowAtEndAlphaToAlphaFunction(alphaSlopeAtStart.getDoubleValue());
      Point2D[] listOfPoints = new Point2D[] {new Point2D(0.0, 0.0), new Point2D(0.6, 0.8), new Point2D(1.0, 1.0)};
      alphaToAlphaFunction = new MultipleSegmentConstantSlope(listOfPoints);



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

      //alphaToAlphaFunction.setSlopeAtStart(alphaSlopeAtStart.getDoubleValue());
      cartesianTrajectoryGenerator.setTrajectoryParameters(swingTime, initialPosition, swingHeight, swingTarget, finalDesiredVelocity);
      cartesianTrajectoryGenerator.initialize();
      timeInStep.set(0.0);

      //New traj generator
      parabolicCartesianTrajectoryGenerator.updateGroundClearance(swingHeight);
      parabolicCartesianTrajectoryGenerator.initialize(initialPosition, zeroVector, zeroVector, swingTarget, finalDesiredVelocity);
   }
   
   public void computeSwing(FramePoint framePointToPack)
   {
      if (USE_NEW_SWING_GENERATOR)
      {
         alphaIn.set(timeInStep.getDoubleValue()/swingTimeDoubleProvider.getValue());
         alphaOut.set(alphaToAlphaFunction.getAlphaPrime(alphaIn.getDoubleValue()));
         alphaTimeInStep.set(swingTimeDoubleProvider.getValue() * alphaOut.getDoubleValue());
         parabolicCartesianTrajectoryGenerator.compute(alphaTimeInStep.getDoubleValue());  //computeNextTick(framePointToPack, dt);
         parabolicCartesianTrajectoryGenerator.getPosition(framePointToPack);
         timeInStep.set(timeInStep.getDoubleValue() + dt);
         updateBagOfBalls(framePointToPack);
      }
      else
      {
         cartesianTrajectoryGenerator.compute(dt);
         cartesianTrajectoryGenerator.getPosition(desiredEndEffectorPosition);
         framePointToPack.setIncludingFrame(desiredEndEffectorPosition);
         updateBagOfBalls(desiredEndEffectorPosition);
      }
   }
   
   public double getTimeRemaining()
   {
      if (USE_NEW_SWING_GENERATOR)
      {
         return swingTimeDoubleProvider.getValue() - timeInStep.getDoubleValue();
      }
      else
      {
         return cartesianTrajectoryGenerator.getTimeRemaining();
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
