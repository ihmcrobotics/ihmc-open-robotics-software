package us.ihmc.manipulation.planning.manifold;

import org.junit.jupiter.api.Test;
import toolbox_msgs.ReachingManifoldMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class ReachingManifoldToolsTest
{
   private static final int iters = 10;

   private final static double positionWeight = 1.0;
   private final static double orientationWeight = 0.0;
   private final static double errorThreshold = 0.001;

   private final RobotSide robotSide = RobotSide.RIGHT;
   private final RigidBodyBasics dummyHand = new RigidBody("dummyHand", new RigidBodyTransform(), ReferenceFrame.getWorldFrame());

   private static final Sphere3DReadOnly sphere = new Sphere3D(3.0, 3.0, 3.0, 1.0);
   private static final Cylinder3DReadOnly cylinder = new Cylinder3D(1.0, 0.5);

   private void appendRandomTransform(RigidBodyTransform transform, Random random)
   {
      transform.appendTranslation(random.nextDouble(), random.nextDouble(), random.nextDouble());
      transform.appendRollRotation(random.nextDouble());
      transform.appendPitchRotation(random.nextDouble());
      transform.appendYawRotation(random.nextDouble());
   }

   @Test
   public void testFindingClosestPointOnSphere()
   {
      List<ReachingManifoldMessage> manifolds = ReachingManifoldTools.createSphereManifoldMessagesForValkyrie(robotSide, dummyHand, sphere);
      List<ReachingManifoldCommand> manifoldCommands = new ArrayList<>();
      for (int i = 0; i < manifolds.size(); i++)
      {
         ReachingManifoldCommand command = new ReachingManifoldCommand();
         command.setFromMessage(manifolds.get(i));
         manifoldCommands.add(command);
      }

      Random random = new Random(1738L);

      for (int i = 0; i < iters; i++)
      {
         RigidBodyTransform expectedClosestTransform = new RigidBodyTransform();
         RigidBodyTransform from = new RigidBodyTransform();
         appendRandomTransform(from, random);

         RigidBodyTransform shapeTransform = new RigidBodyTransform();
         shapeTransform.appendTranslation(1.0, 1.0, 1.0);
         appendRandomTransform(shapeTransform, random);

         System.out.println("distance between manifold and transform = " + ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifoldCommands,
                                                                                                                                         from,
                                                                                                                                         expectedClosestTransform,
                                                                                                                                         positionWeight,
                                                                                                                                         orientationWeight));

         RigidBodyTransform closestTransformToExpectedClosestTransform = new RigidBodyTransform();
         double distance = ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifoldCommands,
                                                                                         expectedClosestTransform,
                                                                                         closestTransformToExpectedClosestTransform,
                                                                                         positionWeight,
                                                                                         orientationWeight);

         assertTrue(distance < errorThreshold,
                    "expected transform shoulds be on the manifolds, with a distance threshold of " + errorThreshold + ". The actual distance is " + distance);
      }
   }

   @Test
   public void testFindingClosestPointOnCylinder()
   {
      Random random = new Random(1738L);

      List<ReachingManifoldMessage> manifolds = ReachingManifoldTools.createCylinderManifoldMessagesForValkyrie(robotSide, dummyHand, cylinder);
      List<ReachingManifoldCommand> manifoldCommands = new ArrayList<>();
      for (int i = 0; i < manifolds.size(); i++)
      {
         ReachingManifoldCommand command = new ReachingManifoldCommand();
         command.setFromMessage(manifolds.get(i));
         manifoldCommands.add(command);
      }

      for (int i = 0; i < iters; i++)
      {
         RigidBodyTransform expectedClosestTransform = new RigidBodyTransform();
         RigidBodyTransform from = new RigidBodyTransform();
         appendRandomTransform(from, random);

         RigidBodyTransform shapeTransform = new RigidBodyTransform();
         shapeTransform.appendTranslation(1.0, 1.0, 1.0);
         appendRandomTransform(shapeTransform, random);

         double distanceBetweenManifoldAndTransform = ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifoldCommands,
                                                                                                                    from,
                                                                                                                    expectedClosestTransform,
                                                                                                                    positionWeight,
                                                                                                                    orientationWeight);
         System.out.println("distance between manifold and transform = " + distanceBetweenManifoldAndTransform);

         RigidBodyTransform closestTransformToExpectedClosestTransform = new RigidBodyTransform();
         double distance = ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifoldCommands,
                                                                                         expectedClosestTransform,
                                                                                         closestTransformToExpectedClosestTransform,
                                                                                         positionWeight,
                                                                                         orientationWeight);

         assertTrue(distance < errorThreshold,
                    "expected transform shoulds be on the manifolds, with a distance threshold of " + errorThreshold + ". The actual distance is " + distance);
      }
   }
}