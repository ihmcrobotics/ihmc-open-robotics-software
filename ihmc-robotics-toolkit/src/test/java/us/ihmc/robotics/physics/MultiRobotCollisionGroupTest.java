package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.*;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

public class MultiRobotCollisionGroupTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testRetrieveCollisionGroups()
   {
      Random random = new Random(4365);

      { // No collision
         List<MultiRobotCollisionGroup> collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(new CollisionListResult());
         assertTrue(collisionGroups.isEmpty());
      }

      { // Single robot single contact
         RigidBodyBasics rootBody = nextRobot(random, "Toto");
         Collidable collidable = nextCollidable(random, rootBody);
         CollisionListResult collisionListResult = new CollisionListResult();
         collisionListResult.add(toCollisionResult(collidable, emptyCollidable()));
         List<MultiRobotCollisionGroup> collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(collisionListResult);

         assertEquals(1, collisionGroups.size());
         MultiRobotCollisionGroup firstGroup = collisionGroups.get(0);
         assertEquals(1, firstGroup.getNumberOfRobots());
         assertTrue(firstGroup.contains(rootBody));
         assertEquals(1, firstGroup.getNumberOfCollisions());
         assertTrue(collisionListResult.get(0) == firstGroup.getGroupCollisions().get(0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Single robot multiple contacts
         RigidBodyBasics rootBody = nextRobot(random, "Toto");
         int numberOfCollisions = random.nextInt(10) + 1;
         CollisionListResult collisionListResult = new CollisionListResult();

         for (int j = 0; j < numberOfCollisions; j++)
         {
            if (random.nextBoolean())
               collisionListResult.add(toCollisionResult(nextCollidable(random, rootBody), emptyCollidable()));
            else
               collisionListResult.add(toCollisionResult(emptyCollidable(), nextCollidable(random, rootBody)));
         }
         List<MultiRobotCollisionGroup> collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(collisionListResult);

         assertEquals(1, collisionGroups.size());
         MultiRobotCollisionGroup firstGroup = collisionGroups.get(0);
         assertEquals(1, firstGroup.getNumberOfRobots());
         assertTrue(firstGroup.contains(rootBody));
         assertEquals(numberOfCollisions, firstGroup.getNumberOfCollisions());
         assertTrue(firstGroup.getGroupCollisions().containsAll(collisionListResult));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Two robots with collisions but not interacting
         RigidBodyBasics rootBody1 = nextRobot(random, "rob1");
         RigidBodyBasics rootBody2 = nextRobot(random, "rob2");

         int numberOfCollisions = random.nextInt(10) + 1;
         CollisionListResult collisionListResult = new CollisionListResult();

         for (int j = 0; j < numberOfCollisions; j++)
         {
            if (random.nextBoolean())
               collisionListResult.add(toCollisionResult(nextCollidable(random, rootBody1), emptyCollidable()));
            else
               collisionListResult.add(toCollisionResult(emptyCollidable(), nextCollidable(random, rootBody1)));

            if (random.nextBoolean())
               collisionListResult.add(toCollisionResult(nextCollidable(random, rootBody2), emptyCollidable()));
            else
               collisionListResult.add(toCollisionResult(emptyCollidable(), nextCollidable(random, rootBody2)));
         }

         List<MultiRobotCollisionGroup> collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(collisionListResult);

         assertEquals(2, collisionGroups.size());
         assertTrue(collisionGroups.stream().allMatch(group -> group.getNumberOfRobots() == 1));
         Set<RigidBodyBasics> allRootBodies = collisionGroups.stream().flatMap(group -> group.getRootBodies().stream()).collect(Collectors.toSet());
         assertTrue(allRootBodies.contains(rootBody1));
         assertTrue(allRootBodies.contains(rootBody2));
         CollisionListResult allCollisions = collisionGroups.stream().flatMap(group -> group.getGroupCollisions().stream())
                                                            .collect(CollisionListResult.collector());
         assertEquals(collisionListResult.size(), allCollisions.size());
         assertTrue(collisionListResult.containsAll(allCollisions));

         for (MultiRobotCollisionGroup group : collisionGroups)
         {
            RigidBodyBasics rootBody = group.getRootBodies().iterator().next();

            for (CollisionResult collision : group.getGroupCollisions())
            {
               assertTrue(collision.getCollidableA().getRootBody() == rootBody || collision.getCollidableB().getRootBody() == rootBody);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Two robots with collisions and interacting
         RigidBodyBasics rootBody1 = nextRobot(random, "rob1");
         RigidBodyBasics rootBody2 = nextRobot(random, "rob2");

         int numberOfSeparateCollisions = random.nextInt(10) + 1;
         CollisionListResult collisionListResult = new CollisionListResult();

         for (int j = 0; j < numberOfSeparateCollisions; j++)
         {
            if (random.nextBoolean())
               collisionListResult.add(toCollisionResult(nextCollidable(random, rootBody1), emptyCollidable()));
            else
               collisionListResult.add(toCollisionResult(emptyCollidable(), nextCollidable(random, rootBody1)));

            if (random.nextBoolean())
               collisionListResult.add(toCollisionResult(nextCollidable(random, rootBody2), emptyCollidable()));
            else
               collisionListResult.add(toCollisionResult(emptyCollidable(), nextCollidable(random, rootBody2)));
         }

         int numberOfInterCollisions = random.nextInt(5) + 1;

         for (int j = 0; j < numberOfInterCollisions; j++)
         {
            if (random.nextBoolean())
               collisionListResult.add(toCollisionResult(nextCollidable(random, rootBody1), nextCollidable(random, rootBody2)));
            else
               collisionListResult.add(toCollisionResult(nextCollidable(random, rootBody2), nextCollidable(random, rootBody1)));
         }

         List<MultiRobotCollisionGroup> collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(collisionListResult);

         assertEquals(1, collisionGroups.size());
         MultiRobotCollisionGroup firstGroup = collisionGroups.get(0);
         assertEquals(2, firstGroup.getNumberOfRobots());
         assertTrue(firstGroup.contains(rootBody1));
         assertTrue(firstGroup.contains(rootBody2));

         CollisionListResult groupCollisions = firstGroup.getGroupCollisions();
         assertEquals(collisionListResult.size(), groupCollisions.size());
         assertTrue(collisionListResult.containsAll(groupCollisions));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Random scenario with N robots, M collisions between robot <=> environment and robot <=> robot.
         int numberOfRobots = random.nextInt(15) + 1;
         List<RigidBodyBasics> rootBodies = nextRobots(random, "blop", numberOfRobots);
         int numberOfCollisions = random.nextInt(50) + 1;

         Set<RigidBodyBasics> collidingRobots = new HashSet<>();
         Set<RigidBodyBasics> notCollidingRobots = new HashSet<>(rootBodies);
         CollisionListResult input = new CollisionListResult();

         for (int j = 0; j < numberOfCollisions; j++)
         {
            if (numberOfRobots == 1 || random.nextBoolean())
            {
               RigidBodyBasics rootBody = nextElementIn(random, rootBodies);
               collidingRobots.add(rootBody);
               notCollidingRobots.remove(rootBody);

               if (random.nextBoolean())
                  input.add(toCollisionResult(nextCollidable(random, rootBody), emptyCollidable()));
               else
                  input.add(toCollisionResult(emptyCollidable(), nextCollidable(random, rootBody)));
            }
            else
            {
               RigidBodyBasics rootA = nextElementIn(random, rootBodies);
               RigidBodyBasics rootB = nextElementIn(random, rootBodies);
               collidingRobots.add(rootA);
               collidingRobots.add(rootB);
               notCollidingRobots.remove(rootA);
               notCollidingRobots.remove(rootB);
               input.add(toCollisionResult(nextCollidable(random, rootA), nextCollidable(random, rootB)));
            }
         }

         List<MultiRobotCollisionGroup> collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(input);

         List<RigidBodyBasics> groupedRobots = collisionGroups.stream().flatMap(group -> group.getRootBodies().stream()).collect(Collectors.toList());
         assertEquals(collidingRobots.size(), groupedRobots.size());
         assertTrue(groupedRobots.containsAll(collidingRobots));
         assertTrue(notCollidingRobots.stream().noneMatch(root -> groupedRobots.contains(root)));
         CollisionListResult groupedCollisions = collisionGroups.stream().flatMap(group -> group.getGroupCollisions().stream())
                                                                .collect(CollisionListResult.collector());
         assertEquals(input.size(), groupedCollisions.size());
         assertTrue(groupedCollisions.containsAll(input));

         for (MultiRobotCollisionGroup group : collisionGroups)
         {
            Set<RigidBodyBasics> rootBodiesFromCollisions = new HashSet<>();
            for (CollisionResult collision : group.getGroupCollisions())
            {
               if (collision.getCollidableA().getRootBody() != null)
                  rootBodiesFromCollisions.add(collision.getCollidableA().getRootBody());
               if (collision.getCollidableB().getRootBody() != null)
                  rootBodiesFromCollisions.add(collision.getCollidableB().getRootBody());
            }

            assertEquals(group.getRootBodies().size(), rootBodiesFromCollisions.size());
            assertTrue(rootBodiesFromCollisions.containsAll(group.getRootBodies()));
         }
      }
   }

   private static List<RigidBodyBasics> nextRobots(Random random, String robotName, int numberOfRobots)
   {
      List<RigidBodyBasics> rootBodies = new ArrayList<>();
      for (int i = 0; i < numberOfRobots; i++)
      {
         rootBodies.add(nextRobot(random, robotName + i));
      }
      return rootBodies;
   }

   private static RigidBodyBasics nextRobot(Random random, String robotName)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RigidBody rootBody = new RigidBody(robotName + "RootBody", worldFrame);
      MultiBodySystemRandomTools.nextJointChain(random, robotName, rootBody, 5);
      return rootBody;
   }

   private static Collidable emptyCollidable()
   {
      return new Collidable(null, -1, -1, null);
   }

   private static Collidable nextCollidable(Random random, RigidBodyBasics rootBody)
   {
      RigidBodyBasics rigidBody = nextElementIn(random, rootBody.subtreeList());
      return new Collidable(rigidBody, -1, -1, null);
   }

   @SafeVarargs
   public static <E> E nextElementIn(Random random, E... elements)
   {
      return elements[random.nextInt(elements.length)];
   }

   public static <E> E nextElementIn(Random random, List<E> list)
   {
      return list.get(random.nextInt(list.size()));
   }

   private static CollisionResult toCollisionResult(Collidable collidableA, Collidable collidableB)
   {
      CollisionResult collisionResult = new CollisionResult();
      collisionResult.setCollidableA(collidableA);
      collisionResult.setCollidableB(collidableB);
      return collisionResult;
   }
}
