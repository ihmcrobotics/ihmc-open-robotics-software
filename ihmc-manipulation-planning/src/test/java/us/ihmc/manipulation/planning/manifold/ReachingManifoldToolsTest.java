package us.ihmc.manipulation.planning.manifold;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;

public class ReachingManifoldToolsTest
{
   private Random random = new Random();

   private final static double positionWeight = 1.0;
   private final static double orientationWeight = 0.0;
   private final static double errorThreshold = 0.001;

   private final RobotSide robotSide = RobotSide.RIGHT;
   private final RigidBodyBasics dummyHand = new RigidBody("dummyHand", new RigidBodyTransform(), ReferenceFrame.getWorldFrame());

   private final RigidBodyTransform shapeTransform = new RigidBodyTransform();
   private final Sphere3D sphere = new Sphere3D(3.0, 3.0, 3.0, 1.0);
   private final Cylinder3D cylinder = new Cylinder3D(1.0, 0.5);

   private void appendRandomTransform(RigidBodyTransform transform)
   {
      transform.appendTranslation(random.nextDouble(), random.nextDouble(), random.nextDouble());
      transform.appendRollRotation(random.nextDouble());
      transform.appendPitchRotation(random.nextDouble());
      transform.appendYawRotation(random.nextDouble());
   }

   @Test
   public void testFindingClosestPointOnSphere()
   {
      RigidBodyTransform expectedClosestTransform = new RigidBodyTransform();
      RigidBodyTransform from = new RigidBodyTransform();
      appendRandomTransform(from);

      shapeTransform.appendTranslation(1.0, 1.0, 1.0);
      appendRandomTransform(shapeTransform);

      List<ReachingManifoldMessage> manifolds = ReachingManifoldTools.createSphereManifoldMessagesForValkyrie(robotSide, dummyHand, sphere);
      List<ReachingManifoldCommand> manifoldCommands = new ArrayList<>();
      for (int i = 0; i < manifolds.size(); i++)
      {
         ReachingManifoldCommand command = new ReachingManifoldCommand();
         command.setFromMessage(manifolds.get(i));
         manifoldCommands.add(command);
      }

      System.out.println("distance between manifold and transform = "
            + ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifoldCommands, from, expectedClosestTransform, positionWeight,
                                                                            orientationWeight));

      RigidBodyTransform closestTransformToExpectedClosestTransform = new RigidBodyTransform();
      double distance = ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifoldCommands, expectedClosestTransform,
                                                                                      closestTransformToExpectedClosestTransform, positionWeight,
                                                                                      orientationWeight);

      System.out.println("distance " + distance);

      assertTrue("expected transform is on the manifolds ", distance < errorThreshold);
   }

   @Test
   public void testFindingClosestPointOnCylinder()
   {
      RigidBodyTransform expectedClosestTransform = new RigidBodyTransform();
      RigidBodyTransform from = new RigidBodyTransform();
      appendRandomTransform(from);

      shapeTransform.appendTranslation(1.0, 1.0, 1.0);
      appendRandomTransform(shapeTransform);

      List<ReachingManifoldMessage> manifolds = ReachingManifoldTools.createCylinderManifoldMessagesForValkyrie(robotSide, dummyHand, cylinder);
      List<ReachingManifoldCommand> manifoldCommands = new ArrayList<>();
      for (int i = 0; i < manifolds.size(); i++)
      {
         ReachingManifoldCommand command = new ReachingManifoldCommand();
         command.setFromMessage(manifolds.get(i));
         manifoldCommands.add(command);
      }

      System.out.println("distance between manifold and transform = "
            + ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifoldCommands, from, expectedClosestTransform, positionWeight,
                                                                            orientationWeight));

      RigidBodyTransform closestTransformToExpectedClosestTransform = new RigidBodyTransform();
      double distance = ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifoldCommands, expectedClosestTransform,
                                                                                      closestTransformToExpectedClosestTransform, positionWeight,
                                                                                      orientationWeight);

      System.out.println("distance " + distance);

      assertTrue("expected transform is on the manifolds ", distance < errorThreshold);
   }
}
