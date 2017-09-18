package us.ihmc.valkyrie.referenceFrames;

import java.lang.reflect.InvocationTargetException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.referenceFrames.ReferenceFrameHashTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReferenceFrameHashTest extends ReferenceFrameHashTest
{
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, true);
   }
   
   @Override
   public void testAddingTwoFramesWithTheSameNameThrowsException()
   {
      super.testAddingTwoFramesWithTheSameNameThrowsException();
   }
   
   @Override
   public void testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList() throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      super.testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList();
   }
   
   @Override
   public void testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList()
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      super.testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList();
   }
   
   @Override
   public void testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode()
   {
      super.testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode();
   }
   
   @Override
   public void testGetReferenceFrameFromHashCodeReturnsSameNamedFrames()
   {
      super.testGetReferenceFrameFromHashCodeReturnsSameNamedFrames();
   }
}
