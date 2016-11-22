package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.junit.Test;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public class BipedalFootstepPlannerNodeTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testEqualsAndHashMethodsWithHardCodedTransforms()
   {
      BipedalFootstepPlannerNode nodeA, nodeB;

      RigidBodyTransform transformA = new RigidBodyTransform();
      RigidBodyTransform transformB = new RigidBodyTransform();

      transformA.setTranslation(1.799, 0.499, 0.001);
      transformB.setTranslation(1.801, 0.501, -0.001);

      nodeA = new BipedalFootstepPlannerNode(RobotSide.LEFT, transformA);
      nodeB = new BipedalFootstepPlannerNode(RobotSide.LEFT, transformB);

      assertTrue(nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() == nodeB.hashCode());

      transformA.setRotationEulerAndZeroTranslation(1.5001, -2.0001, 0.0001);
      transformB.setRotationEulerAndZeroTranslation(1.4999, -1.9999, -0.0001);

      nodeA = new BipedalFootstepPlannerNode(RobotSide.RIGHT, transformA);
      nodeB = new BipedalFootstepPlannerNode(RobotSide.RIGHT, transformB);

      assertTrue(nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() == nodeB.hashCode());

      double positionThreshold = BipedalFootstepPlannerNode.getDistanceThresholdToConsiderNodesEqual();
      double rotationThreshold = BipedalFootstepPlannerNode.getRotationThresholdToConsiderNodesEqual();

      transformA.setTranslation(1.496, -2.52, 0.001);
      transformB.setTranslation(1.496 + 1.01 * positionThreshold, -2.52 + 1.01 * positionThreshold, -0.001 + 1.01 * positionThreshold);

      nodeA = new BipedalFootstepPlannerNode(RobotSide.RIGHT, transformA);
      nodeB = new BipedalFootstepPlannerNode(RobotSide.RIGHT, transformB);

      assertTrue(! nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() != nodeB.hashCode());
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testEqualsAndHashMethodsWithRandomTransforms()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      BipedalFootstepPlannerNode nodeA, nodeB;

      RigidBodyTransform transformA = new RigidBodyTransform();

      for(int i = 0; i < numTrials; i++)
      {
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

         // test for exact same transform
         transformA.setRotationEulerAndZeroTranslation(RandomTools.generateRandomVector(random));
         transformA.setTranslation(RandomTools.generateRandomVector(random));

         nodeA = new BipedalFootstepPlannerNode(robotSide, transformA);
         nodeB = new BipedalFootstepPlannerNode(robotSide, transformA);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testEqualsAndHashcode()
   {
      HashMap<Integer, List<BipedalFootstepPlannerNode>> nodeMap = new HashMap<>();

      System.out.println("left hash: " + RobotSide.LEFT.hashCode());
      System.out.println("right hash: " + RobotSide.RIGHT.hashCode());

      //      for(int pass = 0; pass < 2; pass++)
//      {
//         System.out.println("\n ---- pass " + pass + " ---- ");
//         for(double x = 0; x < 0.2; x += 0.024)
//         {
//            for(double y = 0; y < 0.2; y += 0.024)
//            {
//               System.out.println("trying (" + x + ", " + y + ")");
//
//               RigidBodyTransform transform = new RigidBodyTransform();
//               transform.setTranslation(x, y, 0.0);
//               BipedalFootstepPlannerNode node = new BipedalFootstepPlannerNode(RobotSide.LEFT, transform);
//
//               int hashcode = node.hashCode();
//               if(!nodeMap.containsKey(hashcode))
//               {
//                  List<BipedalFootstepPlannerNode> list = new ArrayList<>();
//                  list.add(node);
//                  nodeMap.put(hashcode, list);
//                  System.out.println("\t accepted");
//               }
//               else
//               {
//                  List<BipedalFootstepPlannerNode> list = nodeMap.get(hashcode);
//                  boolean equalNodeExists = false;
//
//                  for(int i = 0; i < list.size(); i++)
//                  {
//                     if(list.get(i).equals(node))
//                     {
//                        equalNodeExists = true;
//                     }
//                  }
//
//                  if(list.size() > 1)
//                  {
//                     System.out.println("\t\t COLLIDING HASH CODES");
//                  }
//
//                  if(!equalNodeExists)
//                  {
//                     list.add(node);
//                     System.out.println("\t accepted");
//                  }
//                  else
//                  {
//                     System.out.println("\t rejected");
//                  }
//               }
//            }
//         }
//      }
   }
}
