package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import java.util.EnumMap;

import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.JointNames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class SwingTrajectoryGenerator //This is a wrapper class -- it wraps the ParabolicCartesianTrajectoryGenerator into an object of this class
{
   private final double dt;
   private final ParabolicCartesianTrajectoryGenerator cartesianTrajectoryGenerator;
   private final YoVariableDoubleProvider swingTimeDoubleProvider;
   private final YoVariableRegistry registry;
   private final DoubleYoVariable swingHeightParabolaVertix;
   
   private final ReferenceFrame footFrame;
   private final FramePoint currentFootPosition = new FramePoint();
   private final FramePoint desiredEndEffectorPosition = new FramePoint();
   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector initialVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector finalDesiredVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   
   public SwingTrajectoryGenerator(YoVariableRegistry parentRegistry, RobotSide robotSide, double dt, SideDependentList<EnumMap<JointNames, OneDoFJoint>> allLegJoints)
   {
      this.dt = dt;
      footFrame = allLegJoints.get(robotSide).get(JointNames.ANKLE).getFrameAfterJoint(); //TODO make sure this is the one we want
      registry = new YoVariableRegistry("trajectoryRegistry");
      swingTimeDoubleProvider = new YoVariableDoubleProvider("swingTime", registry);
      swingHeightParabolaVertix = new DoubleYoVariable("swingHeightParabolaVertix", registry);
      swingHeightParabolaVertix.set(0.3);    
      cartesianTrajectoryGenerator = new ParabolicCartesianTrajectoryGenerator("swingTrajectory", ReferenceFrame.getWorldFrame(), swingTimeDoubleProvider, swingHeightParabolaVertix.getDoubleValue(), registry);
      parentRegistry.addChild(registry);
   }
   
   public void initializeSwing(double swingTime, FramePoint swingInitial, FramePoint swingTarget)
   {     
      swingTimeDoubleProvider.set(swingTime);
      currentFootPosition.setToZero(footFrame);
      currentFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
      initialPosition.setIncludingFrame(swingInitial);
      
      cartesianTrajectoryGenerator.updateGroundClearance(currentFootPosition.getZ() + swingHeightParabolaVertix.getDoubleValue());
      cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, initialAcceleration, swingTarget, finalDesiredVelocity);
   }

   public void computeSwing(FramePoint framePointToPack)
   {
      cartesianTrajectoryGenerator.computeNextTick(desiredEndEffectorPosition, dt);
      framePointToPack.setIncludingFrame(desiredEndEffectorPosition);
   }
   
   public boolean isDone()
   {
      return cartesianTrajectoryGenerator.isDone();
   }
   
}
