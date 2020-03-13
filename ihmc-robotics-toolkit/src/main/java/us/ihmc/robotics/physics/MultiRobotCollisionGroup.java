package us.ihmc.robotics.physics;

import java.util.*;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class MultiRobotCollisionGroup
{
   private final Set<RigidBodyBasics> rootBodies = new HashSet<>();
   private final CollisionListResult groupCollisions = new CollisionListResult();

   public MultiRobotCollisionGroup()
   {
   }

   public void add(MultiRobotCollisionGroup other)
   {
      rootBodies.addAll(other.rootBodies);
      groupCollisions.addAll(other.groupCollisions);
   }

   public boolean contains(RigidBodyBasics rootBody)
   {
      return rootBodies.contains(rootBody);
   }

   public Set<RigidBodyBasics> getRootBodies()
   {
      return rootBodies;
   }

   public int getNumberOfRobots()
   {
      return rootBodies.size();
   }

   public CollisionListResult getGroupCollisions()
   {
      return groupCollisions;
   }

   public int getNumberOfCollisions()
   {
      return groupCollisions.size();
   }

   @Override
   public String toString()
   {
      return "Root-bodies: " + rootBodies.toString() + ", group collisions:\n" + groupCollisions.toString();
   }

   public static List<MultiRobotCollisionGroup> toCollisionGroups(CollisionListResult allCollisions)
   {
      if (allCollisions.isEmpty())
         return Collections.emptyList();

      List<MultiRobotCollisionGroup> groupList = new ArrayList<>();
      Map<RigidBodyBasics, MultiRobotCollisionGroup> groupMap = new HashMap<>();

      for (CollisionResult collision : allCollisions)
      {
         RigidBodyBasics rootA = collision.getCollidableA().getRootBody();
         RigidBodyBasics rootB = collision.getCollidableB().getRootBody();

         if (rootA == null)
         { // Environment <=> RobotB
            MultiRobotCollisionGroup groupB = groupMap.get(rootB);
            if (groupB == null)
            {
               groupB = new MultiRobotCollisionGroup();
               groupB.rootBodies.add(rootB);
               groupMap.put(rootB, groupB);
               groupList.add(groupB);
            }
            groupB.groupCollisions.add(collision);
         }
         else if (rootB == null)
         { // RobotA <=> Environment
            MultiRobotCollisionGroup groupA = groupMap.get(rootA);
            if (groupA == null)
            {
               groupA = new MultiRobotCollisionGroup();
               groupA.rootBodies.add(rootA);
               groupMap.put(rootA, groupA);
               groupList.add(groupA);
            }
            groupA.groupCollisions.add(collision);
         }
         else
         { // RobotA <=> RobotB
            MultiRobotCollisionGroup groupA = groupMap.get(rootA);
            MultiRobotCollisionGroup groupB = groupMap.get(rootB);

            if (groupA == null)
            {
               if (groupB == null)
               {
                  MultiRobotCollisionGroup groupAB = new MultiRobotCollisionGroup();
                  groupAB.rootBodies.add(rootA);
                  groupAB.rootBodies.add(rootB);
                  groupAB.groupCollisions.add(collision);
                  groupMap.put(rootA, groupAB);
                  groupMap.put(rootB, groupAB);
                  groupList.add(groupAB);
               }
               else
               {
                  MultiRobotCollisionGroup groupAB = groupB;
                  groupAB.rootBodies.add(rootA);
                  groupAB.groupCollisions.add(collision);
                  groupMap.put(rootA, groupAB);
               }
            }
            else if (groupB == null)
            {
               MultiRobotCollisionGroup groupAB = groupA;
               groupAB.rootBodies.add(rootB);
               groupAB.groupCollisions.add(collision);
               groupMap.put(rootB, groupAB);
            }
            else
            {
               if (groupA != groupB)
               {
                  groupA.add(groupB);
                  for (RigidBodyBasics root : groupB.rootBodies)
                     groupMap.put(root, groupA);
                  groupList.remove(groupB);
               }
               groupA.groupCollisions.add(collision);
            }
         }
      }

      return groupList;
   }
}
