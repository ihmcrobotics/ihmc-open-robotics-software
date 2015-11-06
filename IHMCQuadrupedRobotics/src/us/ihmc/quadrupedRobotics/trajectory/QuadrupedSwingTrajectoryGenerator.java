package us.ihmc.quadrupedRobotics.trajectory;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedSwingTrajectoryGenerator
{
   private final double DEFAULT_SWING_TIME = 0.3;
   private final double dt;
   
   private final ParabolicCartesianTrajectoryGenerator cartesianTrajectoryGenerator;
   
   private final RobotQuadrant robotQuadrant;

   private final YoVariableRegistry registry;
   private final DoubleYoVariable swingHeightAboveStartAtFiftyPercent;
   private final YoVariableDoubleProvider swingTimeDoubleProvider;
   
   private final ReferenceFrame soleFrame;
   private final FramePoint currentFootPosition = new FramePoint();
   private final FramePoint desiredEndEffectorPosition = new FramePoint();
   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector initialVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector finalDesiredVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   private BagOfBalls bagOfBalls;
   private int ballCounter = 0;
   
   public QuadrupedSwingTrajectoryGenerator(CommonQuadrupedReferenceFrames referenceFrames, RobotQuadrant robotQuadrant, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, double dt)
   {
      this.dt = dt;
      this.robotQuadrant = robotQuadrant;
      soleFrame = referenceFrames.getFootFrame(robotQuadrant);
      registry = new YoVariableRegistry(robotQuadrant.getCamelCaseNameForStartOfExpression() + "MiniBeastSwingTrajectoryGenerator");
      swingTimeDoubleProvider = new YoVariableDoubleProvider("swingTime", registry);
      swingTimeDoubleProvider.set(DEFAULT_SWING_TIME);
      
      swingHeightAboveStartAtFiftyPercent = new DoubleYoVariable("swingHeightAboveStartAtFiftyPercent", registry);
      swingHeightAboveStartAtFiftyPercent.set(0.1);
      
      ReferenceFrame bodyReferenceFrame = referenceFrames.getBodyFrame();
      
      cartesianTrajectoryGenerator = new ParabolicCartesianTrajectoryGenerator("swingLegTraj", ReferenceFrame.getWorldFrame(), swingTimeDoubleProvider, swingHeightAboveStartAtFiftyPercent.getDoubleValue(), registry);
      parentRegistry.addChild(registry);

      bagOfBalls = new BagOfBalls(50, 0.01, robotQuadrant.getCamelCaseNameForStartOfExpression() + "SwingTrajectoryBoB", registry, yoGraphicsListRegistry);
   }
   
   public void initializeSwing(double swingTime, FramePoint swingInitial, FramePoint swingTarget)
   {
      bagOfBalls.reset();
      
      swingTimeDoubleProvider.set(swingTime);
      currentFootPosition.setToZero(soleFrame);
      currentFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
      initialPosition.setIncludingFrame(swingInitial);
      
      cartesianTrajectoryGenerator.updateGroundClearance(currentFootPosition.getZ() + swingHeightAboveStartAtFiftyPercent.getDoubleValue());
      cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, initialAcceleration, swingTarget, finalDesiredVelocity);
   }
   
   public void computeSwing(FramePoint framePointToPack)
   {
      cartesianTrajectoryGenerator.computeNextTick(desiredEndEffectorPosition, dt);
      framePointToPack.setIncludingFrame(desiredEndEffectorPosition);
      
      updateBagOfBalls(desiredEndEffectorPosition);
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
      return cartesianTrajectoryGenerator.isDone();
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public CartesianTrajectoryGenerator getTrajectoryGenerator()
   {
      return cartesianTrajectoryGenerator;
   }
}
