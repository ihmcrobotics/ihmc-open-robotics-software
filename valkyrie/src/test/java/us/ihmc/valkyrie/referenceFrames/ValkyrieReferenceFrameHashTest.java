package us.ihmc.valkyrie.referenceFrames;

import java.lang.reflect.InvocationTargetException;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.referenceFrames.ReferenceFrameHashTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReferenceFrameHashTest extends ReferenceFrameHashTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 5.0)
   @Test(timeout = 15000, expected = IllegalArgumentException.class)
   public void testAddingTwoFramesWithTheSameNameThrowsException()
   {
      super.testAddingTwoFramesWithTheSameNameThrowsException();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 5.0)
   @Test(timeout = 15000)
   public void testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList() throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      super.testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 5.0)
   @Test(timeout = 15000)
   public void testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList()
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      super.testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 5.0)
   @Test(timeout = 15000)
   public void testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode()
   {
      super.testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 5.0)
   @Test(timeout = 15000)
   public void testGetReferenceFrameFromHashCodeReturnsSameNamedFrames()
   {
      super.testGetReferenceFrameFromHashCodeReturnsSameNamedFrames();
   }
}
