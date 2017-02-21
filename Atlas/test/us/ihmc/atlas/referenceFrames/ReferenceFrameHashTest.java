package us.ihmc.atlas.referenceFrames;

import static org.junit.Assert.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.MemoryTools;

public class ReferenceFrameHashTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 30000)
   public void testGetReferenceFrameFromHashCodeReturnsSameNamedFrames()
   {
      AtlasRobotModel robotModelA = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, true);
      HumanoidReferenceFrames referenceFramesA = new HumanoidReferenceFrames(robotModelA.createFullRobotModel());

      AtlasRobotModel robotModelB = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, true);
      HumanoidReferenceFrames referenceFramesB = new HumanoidReferenceFrames(robotModelB.createFullRobotModel());

      ReferenceFrame midFeetZUpFrameA = referenceFramesA.getMidFeetZUpFrame();
      long nameBasedHashCode = midFeetZUpFrameA.nameBasedHashCode();

      ReferenceFrame midZUpFrameB = referenceFramesB.getReferenceFrameFromNameBaseHashCode(nameBasedHashCode);
      checkReferenceFramesMatch(midFeetZUpFrameA, midZUpFrameB);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 30000)
   public void testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode()
   {
      AtlasRobotModel robotModelA = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, true);
      FullHumanoidRobotModel fullRobotModel = robotModelA.createFullRobotModel();
      HumanoidReferenceFrames referenceFramesA = new HumanoidReferenceFrames(fullRobotModel);

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         ReferenceFrame frameBeforeJoint = joint.getFrameBeforeJoint();
         ReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();
         ReferenceFrame comLinkBefore = joint.getPredecessor().getBodyFixedFrame();
         ReferenceFrame comLinkAfter = joint.getSuccessor().getBodyFixedFrame();

         ReferenceFrame otherFrameBeforeJoint = referenceFramesA.getReferenceFrameFromNameBaseHashCode(frameBeforeJoint.nameBasedHashCode());
         ReferenceFrame otherFrameAfterJoint = referenceFramesA.getReferenceFrameFromNameBaseHashCode(frameAfterJoint.nameBasedHashCode());
         ReferenceFrame otherCoMlinkBefore = referenceFramesA.getReferenceFrameFromNameBaseHashCode(comLinkBefore.nameBasedHashCode());
         ReferenceFrame otherCoMLinkAfter = referenceFramesA.getReferenceFrameFromNameBaseHashCode(comLinkAfter.nameBasedHashCode());

         checkReferenceFramesMatch(frameBeforeJoint, otherFrameBeforeJoint);
         checkReferenceFramesMatch(frameAfterJoint, otherFrameAfterJoint);
         checkReferenceFramesMatch(comLinkBefore, otherCoMlinkBefore);
         checkReferenceFramesMatch(comLinkAfter, otherCoMLinkAfter);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test//(timeout = 30000)
   public void testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList() throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      AtlasRobotModel robotModelA = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, true);
      FullHumanoidRobotModel fullRobotModel = robotModelA.createFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      Method[] declaredMethods = referenceFrames.getClass().getMethods();
      for (Method method : declaredMethods)
      {
         if (method.getReturnType() == ReferenceFrame.class)
         {
            if (method.getParameterCount() == 0)
            {
               ReferenceFrame referenceFrame = (ReferenceFrame) method.invoke(referenceFrames);
               ReferenceFrame referenceFrameFromNameBaseHashCode = referenceFrames.getReferenceFrameFromNameBaseHashCode(referenceFrame.nameBasedHashCode());
               assertNotNull(referenceFrame.getName() + " was not in the reference frame hash map. fix HumanoidReferenceFrames constructor!", referenceFrameFromNameBaseHashCode);
               checkReferenceFramesMatch(referenceFrame, referenceFrameFromNameBaseHashCode);
            }
            else if (method.getParameterCount() == 1 && method.getParameterTypes()[0] == RobotSide.class)
            {
               for(RobotSide robotSide : RobotSide.values)
               {
                  ReferenceFrame referenceFrame = (ReferenceFrame) method.invoke(referenceFrames, robotSide);
                  ReferenceFrame referenceFrameFromNameBaseHashCode = referenceFrames.getReferenceFrameFromNameBaseHashCode(referenceFrame.nameBasedHashCode());
                  assertNotNull("called " + method.getName() + ": " + referenceFrame.getName() + " was not in the reference frame hash map. fix HumanoidReferenceFrames constructor!", referenceFrameFromNameBaseHashCode);
                  checkReferenceFramesMatch(referenceFrame, referenceFrameFromNameBaseHashCode);
               }
            }
         }
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test//(timeout = 30000)
   public void testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList() throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      AtlasRobotModel robotModelA = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, true);
      FullHumanoidRobotModel fullRobotModel = robotModelA.createFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      Method[] declaredMethods = fullRobotModel.getClass().getMethods();
      for (Method method : declaredMethods)
      {
         if (method.getReturnType() == ReferenceFrame.class)
         {
            if (method.getParameterCount() == 0)
            {
               ReferenceFrame referenceFrame = (ReferenceFrame) method.invoke(fullRobotModel);
               ReferenceFrame referenceFrameFromNameBaseHashCode = referenceFrames.getReferenceFrameFromNameBaseHashCode(referenceFrame.nameBasedHashCode());
               assertNotNull(referenceFrame.getName() + " was not in the reference frame hash map. fix HumanoidReferenceFrames constructor!", referenceFrameFromNameBaseHashCode);
               checkReferenceFramesMatch(referenceFrame, referenceFrameFromNameBaseHashCode);
            }
            else if (method.getParameterCount() == 1 && method.getParameterTypes()[0] == RobotSide.class)
            {
               for(RobotSide robotSide : RobotSide.values)
               {
                  ReferenceFrame referenceFrame = (ReferenceFrame) method.invoke(fullRobotModel, robotSide);
                  ReferenceFrame referenceFrameFromNameBaseHashCode = referenceFrames.getReferenceFrameFromNameBaseHashCode(referenceFrame.nameBasedHashCode());
                  assertNotNull("called " + method.getName() + ": " + referenceFrame.getName() + " was not in the reference frame hash map. fix HumanoidReferenceFrames constructor!", referenceFrameFromNameBaseHashCode);
                  checkReferenceFramesMatch(referenceFrame, referenceFrameFromNameBaseHashCode);
               }
            }
         }
      }
   }

   private void checkReferenceFramesMatch(ReferenceFrame referenceFrameA, ReferenceFrame referenceFrameB)
   {
      assertEquals("reference frame names didnt match", referenceFrameA.getName(), referenceFrameB.getName());
      assertEquals("hash codes didn't match", referenceFrameA.nameBasedHashCode(), referenceFrameB.nameBasedHashCode());
      
      if (referenceFrameA.getParent() != null || referenceFrameB.getParent() != null)
      {
         assertEquals("parent reference frame names didnt match", referenceFrameA.getParent().getName(), referenceFrameB.getParent().getName());
         assertEquals("parent hash codes didn't match", referenceFrameA.getParent().nameBasedHashCode(), referenceFrameB.getParent().nameBasedHashCode());
      }
   }

}
