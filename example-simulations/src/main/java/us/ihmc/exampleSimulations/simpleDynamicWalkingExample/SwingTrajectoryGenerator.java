package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import java.util.EnumMap;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.JointNames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class SwingTrajectoryGenerator //This is a wrapper class -- it wraps the ParabolicCartesianTrajectoryGenerator into an object of this class
{
   private final double dt;
   private final ParabolicCartesianTrajectoryGenerator cartesianTrajectoryGenerator;
   private final YoVariableDoubleProvider swingTimeDoubleProvider;
   private final YoVariableRegistry registry;
   private final YoDouble swingHeightParabolaVertix;
   
   private final ReferenceFrame footFrame;
   private final FramePoint3D currentFootPosition = new FramePoint3D();
   private final FramePoint3D desiredEndEffectorPosition = new FramePoint3D();
   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D initialVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D finalDesiredVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   
   public SwingTrajectoryGenerator(YoVariableRegistry parentRegistry, RobotSide robotSide, double dt, SideDependentList<EnumMap<JointNames, OneDoFJoint>> allLegJoints)
   {
      this.dt = dt;
      footFrame = allLegJoints.get(robotSide).get(JointNames.ANKLE).getFrameAfterJoint(); //TODO make sure this is the one we want
      registry = new YoVariableRegistry("trajectoryRegistry");
      swingTimeDoubleProvider = new YoVariableDoubleProvider("swingTime", registry);
      swingHeightParabolaVertix = new YoDouble("swingHeightParabolaVertix", registry);
      swingHeightParabolaVertix.set(0.3);    
      cartesianTrajectoryGenerator = new ParabolicCartesianTrajectoryGenerator("swingTrajectory", ReferenceFrame.getWorldFrame(), swingTimeDoubleProvider, swingHeightParabolaVertix.getDoubleValue(), registry);
      parentRegistry.addChild(registry);
   }
   
   public void initializeSwing(double swingTime, FramePoint3D swingInitial, FramePoint3D swingTarget)
   {     
      swingTimeDoubleProvider.set(swingTime);
      currentFootPosition.setToZero(footFrame);
      currentFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
      initialPosition.setIncludingFrame(swingInitial);
      
      cartesianTrajectoryGenerator.updateGroundClearance(currentFootPosition.getZ() + swingHeightParabolaVertix.getDoubleValue());
      cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, initialAcceleration, swingTarget, finalDesiredVelocity);
   }

   public void computeSwing(FramePoint3D framePointToPack)
   {
      cartesianTrajectoryGenerator.computeNextTick(desiredEndEffectorPosition, dt);
      framePointToPack.setIncludingFrame(desiredEndEffectorPosition);
   }
   
   public boolean isDone()
   {
      return cartesianTrajectoryGenerator.isDone();
   }
   
}
