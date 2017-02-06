package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.LinkedHashMap;

public class CollisionMasksHelper
{
   private final LinkedHashMap<String, ArrayList<? extends CollisionMaskHolder>> groups = new LinkedHashMap<>();
   private final LinkedHashMap<ArrayList<? extends CollisionMaskHolder>, Integer> groupBits = new LinkedHashMap<>();
   private int largestBit = 0x01;

   public void addCollisionGroup(String name, ArrayList<? extends CollisionMaskHolder> group)
   {
      if (largestBit == 0)
      {
         throw new RuntimeException("Number of groups at maximum of 32!");
      }

      this.groups.put(name, group);
      int groupBit = largestBit;
      this.groupBits.put(group, groupBit);

      addToCollisionGroups(group, groupBit);
      largestBit = largestBit << 1;
   }

   public ArrayList<? extends CollisionMaskHolder> getCollisionGroup(String name)
   {
      return groups.get(name);
   }

   public void setToCollideWithGroup(String groupOneName, String groupTwoName)
   {
      ArrayList<? extends CollisionMaskHolder> groupOne = groups.get(groupOneName);
      ArrayList<? extends CollisionMaskHolder> groupTwo = groups.get(groupTwoName);

      Integer bitOne = groupBits.get(groupOne);
      Integer bitTwo = groupBits.get(groupTwo);

      addToCollisionMasks(groupOne, bitTwo);
      addToCollisionMasks(groupTwo, bitOne);
   }

   public void setAsSelfCollidingGroup(String name)
   {
      ArrayList<? extends CollisionMaskHolder> group = groups.get(name);
      Integer groupBit = groupBits.get(group);

      addToCollisionMasks(group, groupBit);
   }

   public void setAsNonSelfCollidingGroup(String name)
   {
      ArrayList<? extends CollisionMaskHolder> group = groups.get(name);
      Integer groupBit = groupBits.get(group);

      removeFromCollisionMasks(group, groupBit);
   }

   private void addToCollisionGroups(ArrayList<? extends CollisionMaskHolder> group, int groupBitsToAddToGroupMasks)
   {
      for (int i = 0; i < group.size(); i++)
      {
         CollisionMaskHolder collisionMaskHolder = group.get(i);
         collisionMaskHolder.setCollisionGroup(collisionMaskHolder.getCollisionGroup() | groupBitsToAddToGroupMasks);
      }
   }

   private void addToCollisionMasks(ArrayList<? extends CollisionMaskHolder> group, Integer groupBitsToAddToCollisionMasks)
   {
      for (int i = 0; i < group.size(); i++)
      {
         CollisionMaskHolder collisionMaskHolder = group.get(i);
         collisionMaskHolder.setCollisionMask(collisionMaskHolder.getCollisionMask() | groupBitsToAddToCollisionMasks);
      }
   }

   private void removeFromCollisionMasks(ArrayList<? extends CollisionMaskHolder> group, Integer groupBitsToRemoveFromCollisionMasks)
   {
      for (int i = 0; i < group.size(); i++)
      {
         CollisionMaskHolder collisionMaskHolder = group.get(i);
         collisionMaskHolder.setCollisionMask(collisionMaskHolder.getCollisionMask() & (0xffffffff ^ groupBitsToRemoveFromCollisionMasks));
      }
   }

}
