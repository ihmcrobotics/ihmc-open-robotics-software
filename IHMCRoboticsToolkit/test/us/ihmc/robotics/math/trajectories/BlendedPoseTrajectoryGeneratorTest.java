package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BlendedPoseTrajectoryGeneratorTest
{
   private final double EPSILON = 1e-3;

   private boolean geometricEquals(FramePose poseA, FramePose poseB, double epsilon)
   {
      return RotationTools.quaternionEpsilonEquals(poseA.getOrientation(), poseB.getOrientation(), epsilon) && poseA.getPosition()
            .epsilonEquals(poseB.getPosition(), epsilon);
   }

   private boolean geometricEquals(FrameOrientation orientationA, FrameOrientation orientationB, double epsilon)
   {
      return RotationTools.quaternionEpsilonEquals(orientationA.getQuaternion(), orientationB.getQuaternion(), epsilon);
   }

   private class PoseTrajectoryState
   {
      public final double time;
      public final FramePoint position;
      public final FrameOrientation orientation;
      public final FrameVector linearVelocity;
      public final FrameVector angularVelocity;
      public final FrameVector linearAcceleration;
      public final FrameVector angularAcceleration;
      private final ReferenceFrame bodyFrame;
      private final ReferenceFrame baseFrame;
      private final ReferenceFrame expressedInFrame;

      public PoseTrajectoryState(PoseTrajectoryGenerator trajectory, double time, ReferenceFrame bodyFrame, ReferenceFrame baseFrame,
            ReferenceFrame expressedInFrame)
      {
         this.position = new FramePoint(expressedInFrame);
         this.orientation = new FrameOrientation(expressedInFrame);
         this.linearVelocity = new FrameVector(expressedInFrame);
         this.angularVelocity = new FrameVector(expressedInFrame);
         this.linearAcceleration = new FrameVector(expressedInFrame);
         this.angularAcceleration = new FrameVector(expressedInFrame);
         this.time = time;
         this.bodyFrame = bodyFrame;
         this.baseFrame = baseFrame;
         this.expressedInFrame = expressedInFrame;
         trajectory.compute(time);
         trajectory.getLinearData(position, linearVelocity, linearAcceleration);
         trajectory.getAngularData(orientation, angularVelocity, angularAcceleration);
      }

      public PoseTrajectoryState(Random random, double time, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
      {
         this.time = time;
         this.position = FramePoint.generateRandomFramePoint(random, expressedInFrame, 1.0, 1.0, 1.0);
         this.orientation = FrameOrientation.generateRandomFrameOrientation(random, expressedInFrame);
         this.linearVelocity = FrameVector.generateRandomFrameVector(random, expressedInFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         this.angularVelocity = FrameVector.generateRandomFrameVector(random, expressedInFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         this.linearAcceleration = FrameVector.generateRandomFrameVector(random, expressedInFrame, -100.0, 100.0, -100.0, 100.0, -100.0, 100.0);
         this.angularAcceleration = FrameVector.generateRandomFrameVector(random, expressedInFrame, -100.0, 100.0, -100.0, 100.0, -100.0, 100.0);
         this.bodyFrame = bodyFrame;
         this.baseFrame = baseFrame;
         this.expressedInFrame = expressedInFrame;
      }

      public FrameSE3TrajectoryPoint getWaypoint()
      {
         return new FrameSE3TrajectoryPoint(time, position, orientation, linearVelocity, angularVelocity);
      }

      public FramePose getPose()
      {
         position.changeFrame(expressedInFrame);
         orientation.changeFrame(expressedInFrame);
         return new FramePose(position, orientation);
      }

      public Twist getTwist()
      {
         linearVelocity.changeFrame(expressedInFrame);
         angularVelocity.changeFrame(expressedInFrame);
         Twist twist = new Twist(bodyFrame, baseFrame, expressedInFrame);
         twist.setLinearPart(linearVelocity);
         twist.setAngularPart(angularVelocity);
         return twist;
      }

      public boolean epsilonEquals(PoseTrajectoryState other, double epsilon)
      {
         return position.epsilonEquals(other.position, epsilon) && geometricEquals(orientation, other.orientation, epsilon) && linearVelocity
               .epsilonEquals(other.linearVelocity, epsilon) && angularVelocity.epsilonEquals(other.angularVelocity, epsilon) && linearAcceleration
               .epsilonEquals(other.linearAcceleration, epsilon) && angularAcceleration.epsilonEquals(other.angularAcceleration, epsilon);
      }
   }

   private PoseTrajectoryGenerator createRandomReferenceTrajectory(Random random, double duration, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      MultipleWaypointsPoseTrajectoryGenerator referenceTrajectory = new MultipleWaypointsPoseTrajectoryGenerator("referenceTrajectory", 10, registry);
      for (int i = 0; i < 10; i++)
      {
         double time = i * duration / (10 - 1);
         PoseTrajectoryState state = new PoseTrajectoryState(random, time, referenceFrame, referenceFrame, referenceFrame);
         referenceTrajectory.appendPoseWaypoint(state.getWaypoint());
      }
      referenceTrajectory.initialize();
      return referenceTrajectory;
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test
   public void testNoConstraints()
   {
      Random random = new Random();
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame bodyFrame = new PoseReferenceFrame("BodyFrame", worldFrame);

      YoVariableRegistry registry = new YoVariableRegistry("trajectory");
      PoseTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, trajectoryDuration, worldFrame, registry);
      BlendedPoseTrajectoryGenerator blendedTrajectory = new BlendedPoseTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Check if blended trajectory is equal to reference trajectory
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         PoseTrajectoryState stateA = new PoseTrajectoryState(referenceTrajectory, time, bodyFrame, worldFrame, worldFrame);
         PoseTrajectoryState stateB = new PoseTrajectoryState(blendedTrajectory, time, bodyFrame, worldFrame, worldFrame);
         assertTrue(stateA.epsilonEquals(stateB, EPSILON));
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test
   public void testInitialPoseConstraint()
   {
      Random random = new Random();
      double initialBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame bodyFrame = new PoseReferenceFrame("BodyFrame", worldFrame);

      YoVariableRegistry registry = new YoVariableRegistry("trajectory");
      PoseTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, trajectoryDuration, worldFrame, registry);
      BlendedPoseTrajectoryGenerator blendedTrajectory = new BlendedPoseTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend initial constraint
      PoseTrajectoryState initialState = new PoseTrajectoryState(random, 0.0, bodyFrame, worldFrame, worldFrame);
      blendedTrajectory.blendInitialConstraint(initialState.getPose(), 0.0, initialBlendDuration);

      // Check if initial pose constraint is satisfied
      PoseTrajectoryState referenceInitialState = new PoseTrajectoryState(referenceTrajectory, 0.0, bodyFrame, worldFrame, worldFrame);
      PoseTrajectoryState blendedInitialState = new PoseTrajectoryState(blendedTrajectory, 0.0, bodyFrame, worldFrame, worldFrame);
      assertTrue(geometricEquals(blendedInitialState.getPose(), initialState.getPose(), EPSILON));
      assertTrue(blendedInitialState.getTwist().epsilonEquals(referenceInitialState.getTwist(), EPSILON));

      // Check if blended trajectory is equal to reference trajectory after the initial blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time > initialBlendDuration)
         {
            PoseTrajectoryState stateA = new PoseTrajectoryState(referenceTrajectory, time, bodyFrame, worldFrame, worldFrame);
            PoseTrajectoryState stateB = new PoseTrajectoryState(blendedTrajectory, time, bodyFrame, worldFrame, worldFrame);
            assertTrue(stateA.epsilonEquals(stateB, EPSILON));
         }
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test
   public void testInitialPoseAndTwistConstraint()
   {
      Random random = new Random();
      double initialBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 100;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame bodyFrame = new PoseReferenceFrame("BodyFrame", worldFrame);

      YoVariableRegistry registry = new YoVariableRegistry("trajectory");
      PoseTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, trajectoryDuration, worldFrame, registry);
      BlendedPoseTrajectoryGenerator blendedTrajectory = new BlendedPoseTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend initial constraint
      PoseTrajectoryState initialState = new PoseTrajectoryState(random, 0.0, bodyFrame, worldFrame, worldFrame);
      blendedTrajectory.blendInitialConstraint(initialState.getPose(), initialState.getTwist(), 0.0, initialBlendDuration);

      // Check if initial pose and twist constraints are satisfied
      PoseTrajectoryState blendedInitialState = new PoseTrajectoryState(blendedTrajectory, 0.0, bodyFrame, worldFrame, worldFrame);
      assertTrue(geometricEquals(blendedInitialState.getPose(), initialState.getPose(), EPSILON));
      assertTrue(blendedInitialState.getTwist().epsilonEquals(initialState.getTwist(), EPSILON));

      // Check if blended trajectory is equal to reference trajectory after the initial blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time > initialBlendDuration)
         {
            PoseTrajectoryState stateA = new PoseTrajectoryState(referenceTrajectory, time, bodyFrame, worldFrame, worldFrame);
            PoseTrajectoryState stateB = new PoseTrajectoryState(blendedTrajectory, time, bodyFrame, worldFrame, worldFrame);
            assertTrue(stateA.epsilonEquals(stateB, EPSILON));
         }
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test
   public void testFinalPoseConstraint()
   {
      Random random = new Random();
      double finalBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame bodyFrame = new PoseReferenceFrame("BodyFrame", worldFrame);

      YoVariableRegistry registry = new YoVariableRegistry("trajectory");
      PoseTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, trajectoryDuration, worldFrame, registry);
      BlendedPoseTrajectoryGenerator blendedTrajectory = new BlendedPoseTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend final constraint
      PoseTrajectoryState finalState = new PoseTrajectoryState(random, trajectoryDuration, bodyFrame, worldFrame, worldFrame);
      blendedTrajectory.blendFinalConstraint(finalState.getPose(), trajectoryDuration, finalBlendDuration);

      // Check if final pose constraint is satisfied
      PoseTrajectoryState referenceFinalState = new PoseTrajectoryState(referenceTrajectory, trajectoryDuration, bodyFrame, worldFrame, worldFrame);
      PoseTrajectoryState blendedFinalState = new PoseTrajectoryState(blendedTrajectory, trajectoryDuration, bodyFrame, worldFrame, worldFrame);
      assertTrue(geometricEquals(blendedFinalState.getPose(), finalState.getPose(), EPSILON));
      assertTrue(blendedFinalState.getTwist().epsilonEquals(referenceFinalState.getTwist(), EPSILON));

      // Check if blended trajectory is equal to reference trajectory before the final blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time < trajectoryDuration - finalBlendDuration)
         {
            PoseTrajectoryState stateA = new PoseTrajectoryState(referenceTrajectory, time, bodyFrame, worldFrame, worldFrame);
            PoseTrajectoryState stateB = new PoseTrajectoryState(blendedTrajectory, time, bodyFrame, worldFrame, worldFrame);
            assertTrue(stateA.epsilonEquals(stateB, EPSILON));
         }
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test
   public void testFinalPoseAndTwistConstraint()
   {
      Random random = new Random();
      double finalBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame bodyFrame = new PoseReferenceFrame("BodyFrame", worldFrame);

      YoVariableRegistry registry = new YoVariableRegistry("trajectory");
      PoseTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, trajectoryDuration, worldFrame, registry);
      BlendedPoseTrajectoryGenerator blendedTrajectory = new BlendedPoseTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend final constraint
      PoseTrajectoryState finalState = new PoseTrajectoryState(random, trajectoryDuration, bodyFrame, worldFrame, worldFrame);
      blendedTrajectory.blendFinalConstraint(finalState.getPose(), finalState.getTwist(), trajectoryDuration, finalBlendDuration);

      // Check if final pose and twist constraint is satisfied
      PoseTrajectoryState blendedFinalState = new PoseTrajectoryState(blendedTrajectory, trajectoryDuration, bodyFrame, worldFrame, worldFrame);
      assertTrue(geometricEquals(blendedFinalState.getPose(), finalState.getPose(), EPSILON));
      assertTrue(blendedFinalState.getTwist().epsilonEquals(finalState.getTwist(), EPSILON));

      // Check if blended trajectory is equal to reference trajectory before the final blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time < trajectoryDuration - finalBlendDuration)
         {
            PoseTrajectoryState stateA = new PoseTrajectoryState(referenceTrajectory, time, bodyFrame, worldFrame, worldFrame);
            PoseTrajectoryState stateB = new PoseTrajectoryState(blendedTrajectory, time, bodyFrame, worldFrame, worldFrame);
            assertTrue(stateA.epsilonEquals(stateB, EPSILON));
         }
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test
   public void testInitialAndFinalConstraint()
   {
      Random random = new Random();
      double initialBlendDuration = 0.25;
      double finalBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame bodyFrame = new PoseReferenceFrame("BodyFrame", worldFrame);

      YoVariableRegistry registry = new YoVariableRegistry("trajectory");
      PoseTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, trajectoryDuration, worldFrame, registry);
      BlendedPoseTrajectoryGenerator blendedTrajectory = new BlendedPoseTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend initial and final constraints
      PoseTrajectoryState initialState = new PoseTrajectoryState(random, 0.0, bodyFrame, worldFrame, worldFrame);
      blendedTrajectory.blendInitialConstraint(initialState.getPose(), initialState.getTwist(), 0.0, initialBlendDuration);

      // Blend final constraint
      PoseTrajectoryState finalState = new PoseTrajectoryState(random, trajectoryDuration, bodyFrame, worldFrame, worldFrame);
      blendedTrajectory.blendFinalConstraint(finalState.getPose(), finalState.getTwist(), trajectoryDuration, finalBlendDuration);

      // Check if initial pose, twist, and acceleration constraints are satisfied
      PoseTrajectoryState blendedInitialState = new PoseTrajectoryState(blendedTrajectory, 0.0, bodyFrame, worldFrame, worldFrame);
      assertTrue(geometricEquals(blendedInitialState.getPose(), initialState.getPose(), EPSILON));
      assertTrue(blendedInitialState.getTwist().epsilonEquals(initialState.getTwist(), EPSILON));

      // Check if final pose, twist, and acceleration constraint is satisfied
      PoseTrajectoryState blendedFinalState = new PoseTrajectoryState(blendedTrajectory, trajectoryDuration, bodyFrame, worldFrame, worldFrame);
      assertTrue(geometricEquals(blendedFinalState.getPose(), finalState.getPose(), EPSILON));
      assertTrue(blendedFinalState.getTwist().epsilonEquals(finalState.getTwist(), EPSILON));

      // Check if blended trajectory is equal to reference trajectory after the initial blend interval and before the final blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time > initialBlendDuration && time < trajectoryDuration - finalBlendDuration)
         {
            PoseTrajectoryState stateA = new PoseTrajectoryState(referenceTrajectory, time, bodyFrame, worldFrame, worldFrame);
            PoseTrajectoryState stateB = new PoseTrajectoryState(blendedTrajectory, time, bodyFrame, worldFrame, worldFrame);
            assertTrue(stateA.epsilonEquals(stateB, EPSILON));
         }
      }
   }
}
