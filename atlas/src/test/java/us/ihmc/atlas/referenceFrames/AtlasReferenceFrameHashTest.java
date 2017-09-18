package us.ihmc.atlas.referenceFrames;

import java.lang.reflect.InvocationTargetException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.referenceFrames.ReferenceFrameHashTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class AtlasReferenceFrameHashTest extends ReferenceFrameHashTest
{
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, true);
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
