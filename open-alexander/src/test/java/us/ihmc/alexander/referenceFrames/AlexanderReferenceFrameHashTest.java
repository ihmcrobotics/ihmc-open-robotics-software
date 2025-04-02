package us.ihmc.alexander.referenceFrames;

import org.junit.jupiter.api.Test;
import us.ihmc.alexander.AlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.referenceFrames.ReferenceFrameHashTest;

import java.lang.reflect.InvocationTargetException;

public class AlexanderReferenceFrameHashTest extends ReferenceFrameHashTest
{
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(AlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
   }
   
   @Override
   @Test
   public void testAddingTwoFramesWithTheSameNameThrowsException()
   {
      super.testAddingTwoFramesWithTheSameNameThrowsException();
   }
   
   @Override
   @Test
   public void testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList() throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      super.testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList();
   }
   
   @Override
   @Test
   public void testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList()
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      super.testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList();
   }
   
   @Override
   @Test
   public void testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode()
   {
      super.testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode();
   }
   
   @Override
   @Test
   public void testGetReferenceFrameFromHashCodeReturnsSameNamedFrames()
   {
      super.testGetReferenceFrameFromHashCodeReturnsSameNamedFrames();
   }
}
