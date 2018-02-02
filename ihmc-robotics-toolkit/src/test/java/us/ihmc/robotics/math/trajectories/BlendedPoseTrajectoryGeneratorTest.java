package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BlendedPoseTrajectoryGeneratorTest
{
   private final double EPSILON = 1e-3;

   private boolean geometricEquals(FramePose3D poseA, FramePose3D poseB, double epsilon)
   {
      return RotationTools.quaternionEpsilonEquals(poseA.getOrientation(), poseB.getOrientation(), epsilon) && poseA.getPosition()
            .epsilonEquals(poseB.getPosition(), epsilon);
   }

   private boolean geometricEquals(FrameQuaternion orientationA, FrameQuaternion orientationB, double epsilon)
   {
      return RotationTools.quaternionEpsilonEquals(orientationA, orientationB, epsilon);
   }

   private class PoseTrajectoryState
   {
      public final double time;
      public final FramePoint3D position;
      public final FrameQuaternion orientation;
      public final FrameVector3D linearVelocity;
      public final FrameVector3D angularVelocity;
      public final FrameVector3D linearAcceleration;
      public final FrameVector3D angularAcceleration;
      private final ReferenceFrame bodyFrame;
      private final ReferenceFrame baseFrame;
      private final ReferenceFrame expressedInFrame;

      public PoseTrajectoryState(PoseTrajectoryGenerator trajectory, double time, ReferenceFrame bodyFrame, ReferenceFrame baseFrame,
            ReferenceFrame expressedInFrame)
      {
         this.position = new FramePoint3D(expressedInFrame);
         this.orientation = new FrameQuaternion(expressedInFrame);
         this.linearVelocity = new FrameVector3D(expressedInFrame);
         this.angularVelocity = new FrameVector3D(expressedInFrame);
         this.linearAcceleration = new FrameVector3D(expressedInFrame);
         this.angularAcceleration = new FrameVector3D(expressedInFrame);
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
         this.position = EuclidFrameRandomTools.nextFramePoint3D(random, expressedInFrame, 1.0, 1.0, 1.0);
         this.orientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expressedInFrame);
         this.linearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expressedInFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         this.angularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expressedInFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         this.linearAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, expressedInFrame, -100.0, 100.0, -100.0, 100.0, -100.0, 100.0);
         this.angularAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, expressedInFrame, -100.0, 100.0, -100.0, 100.0, -100.0, 100.0);
         this.bodyFrame = bodyFrame;
         this.baseFrame = baseFrame;
         this.expressedInFrame = expressedInFrame;
      }

      public FrameSE3TrajectoryPoint getWaypoint()
      {
         return new FrameSE3TrajectoryPoint(time, position, orientation, linearVelocity, angularVelocity);
      }

      public FramePose3D getPose()
      {
         position.changeFrame(expressedInFrame);
         orientation.changeFrame(expressedInFrame);
         return new FramePose3D(position, orientation);
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
   @Test(timeout = 30000)
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
   @Test(timeout = 30000)
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
   @Test(timeout = 30000)
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
   @Test(timeout = 30000)
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
   @Test(timeout = 30000)
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
   @Test(timeout = 30000)
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

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testTroublingDataSet1WithBlending()
   {
      int numberOfSamples = 100;
      double trajectoryDuration = 0.6;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoVariableRegistry registry = new YoVariableRegistry("trajectory");
      MultipleWaypointsPoseTrajectoryGenerator swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator("Swing", 4 + 2, registry);
      BlendedPoseTrajectoryGenerator blendedTrajectory = new BlendedPoseTrajectoryGenerator("blendedTrajectory", swingTrajectory, worldFrame, registry);

      swingTrajectory.clear(worldFrame);

      FramePoint3D initialPosition = new FramePoint3D(worldFrame, -7.212, -0.636, 0.302);
      FrameVector3D initialVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.002);
      FrameQuaternion initialOrientation = new FrameQuaternion(worldFrame, 0.0, -0.0, 1.0, 0.00);

      swingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialVelocity);
      swingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, new FrameVector3D());

      FrameEuclideanTrajectoryPoint firstWaypoint = new FrameEuclideanTrajectoryPoint(0.2, new FramePoint3D(worldFrame, -7.293, -0.623, 0.402),
                                                                                      new FrameVector3D(worldFrame, -1.372, 0.219, 0.475));
      FrameEuclideanTrajectoryPoint secondWaypoint = new FrameEuclideanTrajectoryPoint(0.4, new FramePoint3D(worldFrame, -7.669, -0.563, 0.40),
                                                                                       new FrameVector3D(worldFrame, -1.372, 0.219, 0.425));

      swingTrajectory.appendPositionWaypoint(firstWaypoint);
      swingTrajectory.appendPositionWaypoint(secondWaypoint);


      FramePoint3D finalPosition = new FramePoint3D(worldFrame, -7.75, -0.550, 0.30);
      FrameVector3D finalVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.3);
      FrameQuaternion finalOrientation = new FrameQuaternion(worldFrame, 0.0, -0.0, 1.0, 0.00);

      swingTrajectory.appendPositionWaypoint(trajectoryDuration, finalPosition, finalVelocity);
      swingTrajectory.appendOrientationWaypoint(trajectoryDuration, finalOrientation, new FrameVector3D());

      FramePose3D finalPoseToBlend = new FramePose3D(worldFrame, new FramePoint3D(worldFrame, -7.75, -0.550, 0.3), new FrameQuaternion(worldFrame, 0.0, 0.0, 1.0, 0.0));
      blendedTrajectory.blendFinalConstraint(finalPoseToBlend, 0.6, 0.6);
      blendedTrajectory.initialize();


      FrameQuaternion orientation = new FrameQuaternion(worldFrame);
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);

         blendedTrajectory.compute(time);
         blendedTrajectory.getOrientation(orientation); // this doesn't change throughout the trajectory

         EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(initialOrientation, orientation, 1e-1);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testTroublingDataSet1WithoutBlending()
   {
      int numberOfSamples = 100;
      double trajectoryDuration = 0.6;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoVariableRegistry registry = new YoVariableRegistry("trajectory");
      MultipleWaypointsPoseTrajectoryGenerator swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator("Swing", 4 + 2, registry);
      BlendedPoseTrajectoryGenerator blendedTrajectory = new BlendedPoseTrajectoryGenerator("blendedTrajectory", swingTrajectory, worldFrame, registry);

      swingTrajectory.clear(worldFrame);

      FramePoint3D initialPosition = new FramePoint3D(worldFrame, -7.212, -0.636, 0.302);
      FrameVector3D initialVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.002);
      FrameQuaternion initialOrientation = new FrameQuaternion(worldFrame, 0.0, -0.0, 1.0, 0.005);

      swingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialVelocity);
      swingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, new FrameVector3D());

      FrameEuclideanTrajectoryPoint firstWaypoint = new FrameEuclideanTrajectoryPoint(0.2, new FramePoint3D(worldFrame, -7.293, -0.623, 0.402),
                                                                                      new FrameVector3D(worldFrame, -1.372, 0.219, 0.475));
      FrameEuclideanTrajectoryPoint secondWaypoint = new FrameEuclideanTrajectoryPoint(0.4, new FramePoint3D(worldFrame, -7.669, -0.563, 0.40),
                                                                                       new FrameVector3D(worldFrame, -1.372, 0.219, 0.425));

      swingTrajectory.appendPositionWaypoint(firstWaypoint);
      swingTrajectory.appendPositionWaypoint(secondWaypoint);


      FramePoint3D finalPosition = new FramePoint3D(worldFrame, -7.75, -0.550, 0.30);
      FrameVector3D finalVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.3);
      FrameQuaternion finalOrientation = new FrameQuaternion(worldFrame, 0.0, -0.0, 1.0, 0.00);

      swingTrajectory.appendPositionWaypoint(trajectoryDuration, finalPosition, finalVelocity);
      swingTrajectory.appendOrientationWaypoint(trajectoryDuration, finalOrientation, new FrameVector3D());

      blendedTrajectory.initialize();


      FrameQuaternion orientation = new FrameQuaternion(worldFrame);
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);

         blendedTrajectory.compute(time);
         blendedTrajectory.getOrientation(orientation); // this doesn't change throughout the trajectory

         EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(initialOrientation, orientation, 1e-1);
      }
   }
}
