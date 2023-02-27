package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

public class BlendedPositionTrajectoryGeneratorTest
{
   private final double EPSILON = 1e-3;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   private class PositionTrajectoryState
   {
      public final double time;
      public final FramePoint3D position;
      public final FrameVector3D linearVelocity;
      public final FrameVector3D linearAcceleration;
      private final ReferenceFrame expressedInFrame;

      public PositionTrajectoryState(FixedFramePositionTrajectoryGenerator trajectory, double time,  ReferenceFrame expressedInFrame)
      {
         this.position = new FramePoint3D(expressedInFrame);
         this.linearVelocity = new FrameVector3D(expressedInFrame);
         this.linearAcceleration = new FrameVector3D(expressedInFrame);
         this.time = time;
         this.expressedInFrame = expressedInFrame;
         trajectory.compute(time);
         trajectory.getLinearData(position, linearVelocity, linearAcceleration);
      }

      public PositionTrajectoryState(Random random, double time, ReferenceFrame expressedInFrame)
      {
         this.time = time;
         this.position = EuclidFrameRandomTools.nextFramePoint3D(random, expressedInFrame, 1.0, 1.0, 1.0);
         this.linearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expressedInFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         this.linearAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, expressedInFrame, -100.0, 100.0, -100.0, 100.0, -100.0, 100.0);
         this.expressedInFrame = expressedInFrame;
      }

      public FrameEuclideanTrajectoryPoint getWaypoint()
      {
         return new FrameEuclideanTrajectoryPoint(time, position, linearVelocity);
      }

      public FramePoint3DReadOnly getPosition()
      {
         position.changeFrame(expressedInFrame);
         return new FramePoint3D(position);
      }

      public FrameVector3DReadOnly getLinearVelocity()
      {
         linearVelocity.changeFrame(expressedInFrame);
         return new FrameVector3D(linearVelocity);
      }

      public void assertEpsilonEquals(PositionTrajectoryState other, double epsilon)
      {
         EuclidFrameTestTools.assertGeometricallyEquals(position, other.position, epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(linearVelocity, other.linearVelocity, epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(linearAcceleration, other.linearAcceleration, epsilon);
      }
   }

   private FixedFramePositionTrajectoryGenerator createRandomReferenceTrajectory(Random random, int numberOfWaypoints, double duration, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      MultipleWaypointsPositionTrajectoryGenerator referenceTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("referenceTrajectory", 10, referenceFrame, registry);
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double time = i * duration / (numberOfWaypoints - 1);
         PositionTrajectoryState state = new PositionTrajectoryState(random, time, referenceFrame);
         referenceTrajectory.appendWaypoint(state.getWaypoint());
      }
      referenceTrajectory.initialize();
      return referenceTrajectory;
   }

   @Test
   public void testNoConstraints()
   {
      Random random = new Random();
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      FixedFramePositionTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, 10, trajectoryDuration, worldFrame, registry);
      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Check if blended trajectory is equal to reference trajectory
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         PositionTrajectoryState stateA = new PositionTrajectoryState(referenceTrajectory, time, worldFrame);
         PositionTrajectoryState stateB = new PositionTrajectoryState(blendedTrajectory, time, worldFrame);

         stateA.assertEpsilonEquals(stateB, EPSILON);
      }
   }

   @Test
   public void testInitialPoseConstraint()
   {
      Random random = new Random();
      double initialBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      FixedFramePositionTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, 10, trajectoryDuration, worldFrame, registry);
      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend initial constraint
      PositionTrajectoryState initialState = new PositionTrajectoryState(random, 0.0, worldFrame);
      blendedTrajectory.blendInitialConstraint(initialState.getPosition(), 0.0, initialBlendDuration);

      // Check if initial pose constraint is satisfied
      PositionTrajectoryState referenceInitialState = new PositionTrajectoryState(referenceTrajectory, 0.0, worldFrame);
      PositionTrajectoryState blendedInitialState = new PositionTrajectoryState(blendedTrajectory, 0.0, worldFrame);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedInitialState.getPosition(), initialState.getPosition(), EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedInitialState.getLinearVelocity(), referenceInitialState.getLinearVelocity(), EPSILON);

      // Check if blended trajectory is equal to reference trajectory after the initial blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time > initialBlendDuration)
         {
            PositionTrajectoryState stateA = new PositionTrajectoryState(referenceTrajectory, time, worldFrame);
            PositionTrajectoryState stateB = new PositionTrajectoryState(blendedTrajectory, time, worldFrame);
            stateA.assertEpsilonEquals(stateB, EPSILON);
         }
      }
   }

   @Test
   public void testInitialPoseAndTwistConstraint()
   {
      Random random = new Random();
      double initialBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 100;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame bodyFrame = new PoseReferenceFrame("BodyFrame", worldFrame);

      YoRegistry registry = new YoRegistry("trajectory");
      FixedFramePositionTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, 10, trajectoryDuration, worldFrame, registry);
      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend initial constraint
      PositionTrajectoryState initialState = new PositionTrajectoryState(random, 0.0, worldFrame);
      blendedTrajectory.blendInitialConstraint(initialState.getPosition(), initialState.getLinearVelocity(), 0.0, initialBlendDuration);

      // Check if initial pose and twist constraints are satisfied
      PositionTrajectoryState blendedInitialState = new PositionTrajectoryState(blendedTrajectory, 0.0, worldFrame);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedInitialState.getPosition(), initialState.getPosition(), EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedInitialState.getLinearVelocity(), initialState.getLinearVelocity(), EPSILON);

      // Check if blended trajectory is equal to reference trajectory after the initial blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time > initialBlendDuration)
         {
            PositionTrajectoryState stateA = new PositionTrajectoryState(referenceTrajectory, time, worldFrame);
            PositionTrajectoryState stateB = new PositionTrajectoryState(blendedTrajectory, time, worldFrame);
            stateA.assertEpsilonEquals(stateB, EPSILON);
         }
      }
   }

   @Test
   public void testFinalPositionConstraint()
   {
      Random random = new Random(1738L);
      double finalBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      FixedFramePositionTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, 10, trajectoryDuration, worldFrame, registry);
      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend final constraint
      referenceTrajectory.compute(trajectoryDuration);
      PositionTrajectoryState finalState = new PositionTrajectoryState(random, trajectoryDuration, worldFrame);
      blendedTrajectory.blendFinalConstraint(finalState.getPosition(), trajectoryDuration, finalBlendDuration);

      // Check if final pose constraint is satisfied
      //PoseTrajectoryState referenceFinalState = new PoseTrajectoryState(referenceTrajectory, trajectoryDuration, bodyFrame, worldFrame, worldFrame);
      //PoseTrajectoryState blendedFinalState = new PoseTrajectoryState(blendedTrajectory, trajectoryDuration, bodyFrame, worldFrame, worldFrame);

      FramePoint3D referencePose = new FramePoint3D(worldFrame);
      FramePoint3D blendedPose = new FramePoint3D(worldFrame);

      referenceTrajectory.compute(trajectoryDuration);
      blendedTrajectory.compute(trajectoryDuration);

      referencePose.setIncludingFrame(referenceTrajectory.getPosition());
      blendedPose.setIncludingFrame(blendedTrajectory.getPosition());


      //assertTrue(blendedFinalState.getTwist().epsilonEquals(referenceFinalState.getTwist(), EPSILON));
      EuclidFrameTestTools.assertGeometricallyEquals(finalState.getPosition(), blendedPose, EPSILON);

      // Check if blended trajectory is equal to reference trajectory before the final blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time < trajectoryDuration - finalBlendDuration)
         {
            PositionTrajectoryState stateA = new PositionTrajectoryState(referenceTrajectory, time, worldFrame);
            PositionTrajectoryState stateB = new PositionTrajectoryState(blendedTrajectory, time, worldFrame);
            stateA.assertEpsilonEquals(stateB, EPSILON);
         }
      }
   }

   @Test
   public void testSameFinalPoseConstraint()
   {
      Random random = new Random(1738L);
      double finalBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      double dt = 0.01;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      FixedFramePositionTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, 10, trajectoryDuration, worldFrame, registry);
      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend final constraint
      FramePoint3D finalReferencePosition = new FramePoint3D(worldFrame);
      referenceTrajectory.compute(trajectoryDuration);
      finalReferencePosition.setIncludingFrame(referenceTrajectory.getPosition());
      blendedTrajectory.blendFinalConstraint(finalReferencePosition, trajectoryDuration, trajectoryDuration);
      blendedTrajectory.initialize();

      // Check if final pose constraint is satisfied
      PositionTrajectoryState referenceFinalState = new PositionTrajectoryState(referenceTrajectory, trajectoryDuration, worldFrame);
      PositionTrajectoryState blendedFinalState = new PositionTrajectoryState(blendedTrajectory, trajectoryDuration, worldFrame);

      FramePoint3D referencePosition = new FramePoint3D(worldFrame);
      FramePoint3D blendedPosition = new FramePoint3D(worldFrame);

      referenceTrajectory.compute(trajectoryDuration);
      blendedTrajectory.compute(trajectoryDuration);

      referencePosition.setIncludingFrame(referenceTrajectory.getPosition());
      blendedPosition.setIncludingFrame(blendedTrajectory.getPosition());


      EuclidFrameTestTools.assertGeometricallyEquals(blendedFinalState.getLinearVelocity(), referenceFinalState.getLinearVelocity(), EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(finalReferencePosition, blendedPosition, EPSILON);

      // Check if blended trajectory is equal to reference trajectory for the whole thing
      for (double time = 0.0; time <= trajectoryDuration; time += dt)
      {
         PositionTrajectoryState stateA = new PositionTrajectoryState(referenceTrajectory, time, worldFrame);
         PositionTrajectoryState stateB = new PositionTrajectoryState(blendedTrajectory, time, worldFrame);
         stateA.assertEpsilonEquals(stateB, EPSILON);
      }
   }

   @Test
   public void testFinalPoseAndTwistConstraint()
   {
      Random random = new Random();
      double finalBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      FixedFramePositionTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, 10, trajectoryDuration, worldFrame, registry);
      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend final constraint
      PositionTrajectoryState finalState = new PositionTrajectoryState(random, trajectoryDuration, worldFrame);
      blendedTrajectory.blendFinalConstraint(finalState.getPosition(), finalState.getLinearVelocity(), trajectoryDuration, finalBlendDuration);

      // Check if final pose and twist constraint is satisfied
      PositionTrajectoryState blendedFinalState = new PositionTrajectoryState(blendedTrajectory, trajectoryDuration, worldFrame);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedFinalState.getPosition(), finalState.getPosition(), EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedFinalState.getLinearVelocity(), finalState.getLinearVelocity(), EPSILON);

      // Check if blended trajectory is equal to reference trajectory before the final blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time < trajectoryDuration - finalBlendDuration)
         {
            PositionTrajectoryState stateA = new PositionTrajectoryState(referenceTrajectory, time, worldFrame);
            PositionTrajectoryState stateB = new PositionTrajectoryState(blendedTrajectory, time, worldFrame);
            stateA.assertEpsilonEquals(stateB, EPSILON);
         }
      }
   }

   @Test
   public void testInitialAndFinalConstraint()
   {
      Random random = new Random(1738L);
      double initialBlendDuration = 0.25;
      double finalBlendDuration = 0.25;
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      FixedFramePositionTrajectoryGenerator referenceTrajectory = createRandomReferenceTrajectory(random, 10, trajectoryDuration, worldFrame, registry);
      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", referenceTrajectory, worldFrame, registry);

      // Blend initial constraints
      PositionTrajectoryState initialState = new PositionTrajectoryState(random, 0.0, worldFrame);
      blendedTrajectory.blendInitialConstraint(initialState.getPosition(), initialState.getLinearVelocity(), 0.0, initialBlendDuration);

      // Blend final constraint
      PositionTrajectoryState finalState = new PositionTrajectoryState(random, trajectoryDuration, worldFrame);
      blendedTrajectory.blendFinalConstraint(finalState.getPosition(), finalState.getLinearVelocity(), trajectoryDuration, finalBlendDuration);

      // Check if initial pose, twist, and acceleration constraints are satisfied
      PositionTrajectoryState blendedInitialState = new PositionTrajectoryState(blendedTrajectory, 0.0, worldFrame);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedInitialState.getPosition(), initialState.getPosition(), EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedInitialState.getLinearVelocity(), initialState.getLinearVelocity(), EPSILON);

      // Check if final pose, twist, and acceleration constraint is satisfied
      PositionTrajectoryState blendedFinalState = new PositionTrajectoryState(blendedTrajectory, trajectoryDuration, worldFrame);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedFinalState.getPosition(), finalState.getPosition(), EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(blendedFinalState.getLinearVelocity(), finalState.getLinearVelocity(), EPSILON);

      // Check if blended trajectory is equal to reference trajectory after the initial blend interval and before the final blend interval
      for (int i = 0; i < numberOfSamples; i++)
      {
         double time = i * trajectoryDuration / (numberOfSamples - 1);
         if (time > initialBlendDuration && time < trajectoryDuration - finalBlendDuration)
         {
            PositionTrajectoryState stateA = new PositionTrajectoryState(referenceTrajectory, time, worldFrame);
            PositionTrajectoryState stateB = new PositionTrajectoryState(blendedTrajectory, time, worldFrame);
            stateA.assertEpsilonEquals(stateB, EPSILON);
         }
      }
   }


   @Test
   public void testTroublingDataSet1WithBlending()
   {
      double trajectoryDuration = 0.6;
      double dt = 0.01;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      MultipleWaypointsPositionTrajectoryGenerator swingTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("Swing", 4 + 2, worldFrame, registry);

      swingTrajectory.clear(worldFrame);

      FramePoint3D initialPosition = new FramePoint3D(worldFrame, -7.212, -0.636, 0.302);
      FrameVector3D initialVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.002);

      swingTrajectory.appendWaypoint(0.0, initialPosition, initialVelocity);

      FrameEuclideanTrajectoryPoint firstWaypoint = new FrameEuclideanTrajectoryPoint(0.2, new FramePoint3D(worldFrame, -7.293, -0.623, 0.402),
                                                                                      new FrameVector3D(worldFrame, -1.372, 0.219, 0.475));
      FrameEuclideanTrajectoryPoint secondWaypoint = new FrameEuclideanTrajectoryPoint(0.4, new FramePoint3D(worldFrame, -7.669, -0.563, 0.40),
                                                                                       new FrameVector3D(worldFrame, -1.372, 0.219, 0.425));

      swingTrajectory.appendWaypoint(firstWaypoint);
      swingTrajectory.appendWaypoint(secondWaypoint);


      FramePoint3D finalPosition = new FramePoint3D(worldFrame, -7.75, -0.550, 0.30);
      FrameVector3D finalVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.3);

      swingTrajectory.appendWaypoint(trajectoryDuration, finalPosition, finalVelocity);

      FramePoint3D finalPositionToBlend = new FramePoint3D(worldFrame, -7.75, -0.550, 0.3);

      swingTrajectory.initialize();

      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", swingTrajectory, worldFrame, registry);
      blendedTrajectory.blendFinalConstraint(finalPositionToBlend, trajectoryDuration, trajectoryDuration);
      blendedTrajectory.initialize();

      FramePoint3D tempPosition = new FramePoint3D();
      swingTrajectory.compute(trajectoryDuration);
      tempPosition.setIncludingFrame(swingTrajectory.getPosition());
      EuclidFrameTestTools.assertGeometricallyEquals(finalPosition, tempPosition, EPSILON);

      blendedTrajectory.compute(trajectoryDuration);
      tempPosition.setIncludingFrame(blendedTrajectory.getPosition());
      EuclidFrameTestTools.assertGeometricallyEquals(finalPositionToBlend, tempPosition, EPSILON);
   }

   @Test
   public void testTroublingDataSet1WithoutBlending()
   {
      int numberOfSamples = 100;
      double trajectoryDuration = 0.6;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      MultipleWaypointsPositionTrajectoryGenerator swingTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("Swing", 4 + 2, worldFrame, registry);
      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", swingTrajectory, worldFrame, registry);

      swingTrajectory.clear(worldFrame);

      FramePoint3D initialPosition = new FramePoint3D(worldFrame, -7.212, -0.636, 0.302);
      FrameVector3D initialVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.002);

      swingTrajectory.appendWaypoint(0.0, initialPosition, initialVelocity);

      FrameEuclideanTrajectoryPoint firstWaypoint = new FrameEuclideanTrajectoryPoint(0.2, new FramePoint3D(worldFrame, -7.293, -0.623, 0.402),
                                                                                      new FrameVector3D(worldFrame, -1.372, 0.219, 0.475));
      FrameEuclideanTrajectoryPoint secondWaypoint = new FrameEuclideanTrajectoryPoint(0.4, new FramePoint3D(worldFrame, -7.669, -0.563, 0.40),
                                                                                       new FrameVector3D(worldFrame, -1.372, 0.219, 0.425));

      swingTrajectory.appendWaypoint(firstWaypoint);
      swingTrajectory.appendWaypoint(secondWaypoint);


      FramePoint3D finalPosition = new FramePoint3D(worldFrame, -7.75, -0.550, 0.30);
      FrameVector3D finalVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.3);

      swingTrajectory.appendWaypoint(trajectoryDuration, finalPosition, finalVelocity);

      blendedTrajectory.initialize();
   }

   @Test
   public void testTroublingDataSet2WithBlending()
   {
      double dt = 0.01;
      double trajectoryDuration = 0.6;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry registry = new YoRegistry("trajectory");
      MultipleWaypointsPositionTrajectoryGenerator swingTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("Swing", 4 + 2, worldFrame, registry);

      swingTrajectory.clear(worldFrame);

      FramePoint3D initialPosition = new FramePoint3D(worldFrame, -7.212, -0.636, 0.302);
      FrameVector3D initialVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.002);

      swingTrajectory.appendWaypoint(0.0, initialPosition, initialVelocity);

      FrameEuclideanTrajectoryPoint firstWaypoint = new FrameEuclideanTrajectoryPoint(0.2, new FramePoint3D(worldFrame, -7.293, -0.623, 0.402),
                                                                                      new FrameVector3D(worldFrame, -1.372, 0.219, 0.475));
      FrameEuclideanTrajectoryPoint secondWaypoint = new FrameEuclideanTrajectoryPoint(0.4, new FramePoint3D(worldFrame, -7.669, -0.563, 0.40),
                                                                                       new FrameVector3D(worldFrame, -1.372, 0.219, 0.425));

      swingTrajectory.appendWaypoint(firstWaypoint);
      swingTrajectory.appendWaypoint(secondWaypoint);


      FramePoint3D finalPosition = new FramePoint3D(worldFrame, -7.75, -0.550, 0.30);
      FrameVector3D finalVelocity = new FrameVector3D(worldFrame, -0.0, -0.0, -0.3);

      swingTrajectory.appendWaypoint(trajectoryDuration, finalPosition, finalVelocity);
      swingTrajectory.initialize();

      BlendedPositionTrajectoryGenerator blendedTrajectory = new BlendedPositionTrajectoryGenerator("blendedTrajectory", swingTrajectory, worldFrame, registry);
      FramePoint3D initialPositionToBlend = new FramePoint3D(worldFrame, initialPosition);
      blendedTrajectory.blendInitialConstraint(initialPositionToBlend, 0.0, 0.2);
      blendedTrajectory.initialize();


      for (double time = 0.0; time <= trajectoryDuration; time += dt)
      {
         PositionTrajectoryState stateA = new PositionTrajectoryState(swingTrajectory, time, worldFrame);
         PositionTrajectoryState stateB = new PositionTrajectoryState(blendedTrajectory, time, worldFrame);
         stateA.assertEpsilonEquals(stateB, EPSILON);
      }
   }

}
