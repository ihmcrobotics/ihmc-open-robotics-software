package us.ihmc.robotics.robotDescription;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.MutationTestingTools;

public class CollisionMasksHelperTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCollisionMasksHelper()
   {
      CollisionMeshDescription objectOne = new CollisionMeshDescription();
      CollisionMeshDescription objectTwo = new CollisionMeshDescription();
      CollisionMeshDescription objectThree = new CollisionMeshDescription();
      CollisionMeshDescription objectFour = new CollisionMeshDescription();
      CollisionMeshDescription objectFive = new CollisionMeshDescription();
      CollisionMeshDescription objectSix = new CollisionMeshDescription();

      CollisionMasksHelper helper = new CollisionMasksHelper();
      assertEquals(1, helper.getNextGroupBitMask());

      ArrayList<CollisionMaskHolder> groupOne = new ArrayList<>();

      groupOne.add(objectOne);
      groupOne.add(objectTwo);

      ArrayList<CollisionMaskHolder> groupTwo = new ArrayList<>();
      groupTwo.add(objectThree);
      groupTwo.add(objectFour);

      helper.addCollisionGroup("GroupOne", groupOne);
      helper.addCollisionGroup("GroupTwo", groupTwo);

      assertEquals(0b01, objectOne.getCollisionGroup());
      assertEquals(0b01, objectTwo.getCollisionGroup());

      assertEquals(0b10, objectThree.getCollisionGroup());
      assertEquals(0b10, objectFour.getCollisionGroup());

      assertEquals(0b00, objectOne.getCollisionMask());
      assertEquals(0b00, objectTwo.getCollisionMask());

      assertEquals(0b00, objectThree.getCollisionMask());
      assertEquals(0b00, objectFour.getCollisionMask());

      helper.setAsSelfCollidingGroup("GroupOne");
      assertEquals(0b01, objectOne.getCollisionMask());
      assertEquals(0b01, objectTwo.getCollisionMask());
      assertEquals(0b00, objectThree.getCollisionMask());
      assertEquals(0b00, objectFour.getCollisionMask());

      helper.setAsSelfCollidingGroup("GroupTwo");
      assertEquals(0b01, objectOne.getCollisionMask());
      assertEquals(0b01, objectTwo.getCollisionMask());
      assertEquals(0b10, objectThree.getCollisionMask());
      assertEquals(0b10, objectFour.getCollisionMask());

      helper.setAsNonSelfCollidingGroup("GroupTwo");
      assertEquals(0b01, objectOne.getCollisionMask());
      assertEquals(0b01, objectTwo.getCollisionMask());
      assertEquals(0b00, objectThree.getCollisionMask());
      assertEquals(0b00, objectFour.getCollisionMask());

      ArrayList<CollisionMaskHolder> groupThree = new ArrayList<>();
      groupThree.add(objectOne);
      groupThree.add(objectFour);
      helper.addCollisionGroup("GroupThree", groupThree);

      assertEquals(0b101, objectOne.getCollisionGroup());
      assertEquals(0b001, objectTwo.getCollisionGroup());

      assertEquals(0b010, objectThree.getCollisionGroup());
      assertEquals(0b110, objectFour.getCollisionGroup());

      ArrayList<CollisionMaskHolder> groupFour = new ArrayList<>();
      groupFour.add(objectFive);
      groupFour.add(objectSix);
      helper.addCollisionGroup("GroupFour", groupFour);
      assertEquals(0b1000, objectFive.getCollisionGroup());
      assertEquals(0b1000, objectFive.getCollisionGroup());

      ArrayList<CollisionMaskHolder> groupFive = new ArrayList<>();
      groupFive.add(objectSix);
      helper.addCollisionGroup("GroupFive", groupFive);
      assertEquals(32, helper.getNextGroupBitMask());
      assertEquals(0b11000, objectSix.getCollisionGroup());

      assertEquals(groupOne, helper.getCollisionGroup("GroupOne"));
      assertEquals(groupTwo, helper.getCollisionGroup("GroupTwo"));
      assertEquals(groupThree, helper.getCollisionGroup("GroupThree"));
      assertEquals(groupFour, helper.getCollisionGroup("GroupFour"));
      assertEquals(groupFive, helper.getCollisionGroup("GroupFive"));

      helper.setToCollideWithGroup("GroupFour", "GroupTwo");
      assertEquals(0b1000, objectThree.getCollisionMask());
      assertEquals(0b1000, objectFour.getCollisionMask());
      assertEquals(0b010, objectFive.getCollisionMask());
      assertEquals(0b010, objectSix.getCollisionMask());

      helper.setAsSelfCollidingGroup("GroupFour");
      assertEquals(0b1000, objectThree.getCollisionMask());
      assertEquals(0b1000, objectFour.getCollisionMask());
      assertEquals(0b1010, objectFive.getCollisionMask());
      assertEquals(0b1010, objectSix.getCollisionMask());

      helper.setAsNonSelfCollidingGroup("GroupFour");
      assertEquals(0b1000, objectThree.getCollisionMask());
      assertEquals(0b1000, objectFour.getCollisionMask());
      assertEquals(0b010, objectFive.getCollisionMask());
      assertEquals(0b010, objectSix.getCollisionMask());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMaxNumberOfGroupsInCollisionMasksHelper()
   {
      CollisionMasksHelper helper = new CollisionMasksHelper();

      for (int i = 0; i < 32; i++)
      {
         ArrayList<CollisionMaskHolder> group = new ArrayList<>();
         helper.addCollisionGroup("Group" + i, group);
      }

      ArrayList<CollisionMaskHolder> group32 = new ArrayList<>();

      CollisionMeshDescription object32 = new CollisionMeshDescription();
      group32.add(object32);

      try
      {
         helper.addCollisionGroup("Group32", group32);
         fail("Too many groups");
      }
      catch (Exception e)
      {
         assertEquals("Number of groups at maximum of 32!", e.getMessage());
      }
   }

   public static void main(String[] args)
   {
      String targetTests = CollisionMasksHelperTest.class.getName();
      String targetClassesInSamePackage = CollisionMasksHelper.class.getName();
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
