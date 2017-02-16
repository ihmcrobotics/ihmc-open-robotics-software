package us.ihmc.robotics.referenceFrames;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
//import us.ihmc.utilities.math.geometry.Transform3D;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commons.Assertions;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RigidBodyTransformTest;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.JUnitTools;

public class ReferenceFrameTest
{
   private static final boolean VERBOSE = false;

   private ReferenceFrame root, frame1, frame2, frame3, frame4, frame5, frame6, frame7, frame8;
   private ReferenceFrame root2, frame9, frame10, frame11;
   private ReferenceFrame[] frames1, frames2;

   private ArrayList<ReferenceFrame> allFramesTogether;

   private LinkedHashMap<String, RigidBodyTransform> transformsForVerification;

   public void setUp()
   {
      transformsForVerification = new LinkedHashMap<String, RigidBodyTransform>();

      // The structure we'll test is as follows:
      // root                                                 root2
      // frame1                frame2                                    frame9
      // frame3   frame4        frame6  frame7                            frame10 frame11
      // frame5                frame8

      root = ReferenceFrame.constructARootFrame("root");

      // Some are randomly changing and some are random but unchanging:
      frame1 = constructRandomUnchangingFrame("frame1", root);
      frame2 = new RandomlyChangingFrame("frame2", root);
      frame3 = constructRandomUnchangingFrame("frame3", root);
      frame4 = new RandomlyChangingFrame("frame4", frame1);
      frame5 = constructRandomUnchangingFrame("frame5", root);
      frame6 = new RandomlyChangingFrame("frame6", frame2);
      frame7 = constructRandomUnchangingFrame("frame7", root);
      frame8 = new RandomlyChangingFrame("frame8", frame7);

      root2 = ReferenceFrame.constructARootFrame("root2");
      frame9 = constructRandomUnchangingFrame("frame9", root2);
      frame10 = new RandomlyChangingFrame("frame10", frame9);
      frame11 = constructRandomUnchangingFrame("frame11", frame9);

      frames1 = new ReferenceFrame[] { root, frame1, frame2, frame3, frame4, frame5, frame6, frame7, frame8 };
      frames2 = new ReferenceFrame[] { root2, frame9, frame10, frame11 };

      allFramesTogether = new ArrayList<ReferenceFrame>();
      addAllFrames(allFramesTogether, frames1);
      addAllFrames(allFramesTogether, frames2);
   }

   private void addAllFrames(ArrayList<ReferenceFrame> arrayList, ReferenceFrame[] frames)
   {
      for (ReferenceFrame frame : frames)
      {
         arrayList.add(frame);
      }
   }

   public void tearDown()
   {
      transformsForVerification = null;
      frames1 = frames2 = null;
      root = frame1 = frame2 = frame3 = frame4 = frame5 = frame6 = frame7 = frame8 = null;
      root2 = frame9 = frame10 = frame11 = null;
      allFramesTogether = null;
   }

   private ReferenceFrame constructRandomUnchangingFrame(String nameOfFrame, ReferenceFrame parentOfFrame)
   {
      RigidBodyTransform randomTransformToParent = generateRandomTransform();
      transformsForVerification.put(nameOfFrame, new RigidBodyTransform(randomTransformToParent));
      ReferenceFrame ret = ReferenceFrame.constructFrameWithUnchangingTransformToParent(nameOfFrame, parentOfFrame, randomTransformToParent);

      return ret;

   }

   private class RandomlyChangingFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -476837045790926369L;

      public RandomlyChangingFrame(String frameName, ReferenceFrame parentFrame)
      {
         super(frameName, parentFrame, false, false, false);
      }

      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         RigidBodyTransform randomTransform = generateRandomTransform();
         transformToParent.set(randomTransform);

         transformsForVerification.put(this.getName(), new RigidBodyTransform(randomTransform));
      }

   }

   private RigidBodyTransform generateRandomTransform()
   {
         Random random = new Random();
         Matrix3d rotX = new Matrix3d();
         Matrix3d rotY = new Matrix3d();
         Matrix3d rotZ = new Matrix3d();
         Vector3d trans = new Vector3d();

         randomizeVector(random, trans);
         createRandomRotationMatrixX(random, rotX);
         createRandomRotationMatrixY(random, rotY);
         createRandomRotationMatrixZ(random, rotZ);

         rotX.mul(rotY);
         rotX.mul(rotZ);
         
         RigidBodyTransform ret = new RigidBodyTransform(rotX, trans);
         return ret;
   }
   
   private void randomizeVector(Random random, Vector3d vector)
   {
      vector.setX(random.nextDouble());
      vector.setY(random.nextDouble());
      vector.setZ(random.nextDouble());
   }
   
   private void createRandomRotationMatrixX(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.setM00(1);
      matrix.setM01(0);
      matrix.setM02(0);
      matrix.setM10(0);
      matrix.setM11(cTheta);
      matrix.setM12(-sTheta);
      matrix.setM20(0);
      matrix.setM21(sTheta);
      matrix.setM22(cTheta);
   }

   private void createRandomRotationMatrixY(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.setM00(cTheta);
      matrix.setM01(0);
      matrix.setM02(sTheta);
      matrix.setM10(0);
      matrix.setM11(1);
      matrix.setM12(0);
      matrix.setM20(-sTheta);
      matrix.setM21(0);
      matrix.setM22(cTheta);
   }

   private void createRandomRotationMatrixZ(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.setM00(cTheta);
      matrix.setM01(-sTheta);
      matrix.setM02(0);
      matrix.setM10(sTheta);
      matrix.setM11(cTheta);
      matrix.setM12(0);
      matrix.setM20(0);
      matrix.setM21(0);
      matrix.setM22(1);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTypicalExample()
   {
      setUp();

      RigidBodyTransform transformFrom9To10 = frame9.getTransformToDesiredFrame(frame10);
      RigidBodyTransform transformFrom10To9 = frame10.getTransformToDesiredFrame(frame9);

      RigidBodyTransform shouldBeIdentity = new RigidBodyTransform(transformFrom10To9);
      shouldBeIdentity.multiply(transformFrom9To10);

      assertEquals(1.0, shouldBeIdentity.determinantRotationPart(), 1e-7);

      tearDown();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTransformToParents()
   {
      setUp();
      updateAllFrames();

      for (ReferenceFrame frame : allFramesTogether)
      {
         frame.checkRepInvariants();

         ReferenceFrame parent = frame.getParent();
         if (parent != null)
         {
            RigidBodyTransform transformToParentOne = frame.getTransformToParent();
            RigidBodyTransform transformToParentTwo = frame.getTransformToDesiredFrame(parent);
            RigidBodyTransform transformToParentThree = transformsForVerification.get(frame.getName());

            verifyTransformsAreEpsilonEqual(transformToParentOne, transformToParentTwo, transformToParentThree);
         }
      }
      tearDown();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTransformToRoots()
   {
      setUp();
      updateAllFrames();

      for (ReferenceFrame frame : frames1)
      {
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(root);
         verifyTransformToRootByClimbingTree(frame, transformToRootOne);
      }

      for (ReferenceFrame frame : frames2)
      {
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(root2);
         verifyTransformToRootByClimbingTree(frame, transformToRootOne);
      }
      tearDown();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void getTransformToSelf()
   {
      setUp();
      updateAllFrames();

      for (ReferenceFrame frame : allFramesTogether)
      {
         RigidBodyTransform transformToSelf = frame.getTransformToDesiredFrame(frame);
         verifyTransformsAreEpsilonEqual(transformToSelf, new RigidBodyTransform());
      }
      tearDown();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testGetTransformBetweenFrames()
   {
      setUp();
      Random random = new Random(1776L);
      updateAllFrames();

      int numberOfTests = 100000;
      int numberTestsComplete = 0;

      for (int i = 0; i < numberOfTests; i++)
      {
         ReferenceFrame frame1 = selectARandomFrame(random);
         ReferenceFrame frame2 = selectARandomFrame(random);

         updateARandomFrame(random);

         if (frame1.getRootFrame() != frame2.getRootFrame())
         {
            continue;
         }

         RigidBodyTransform transformOne = frame1.getTransformToDesiredFrame(frame2);
         RigidBodyTransform transformTwo = frame2.getTransformToDesiredFrame(frame1);
         
         transformTwo.invert();

         verifyTransformsAreEpsilonEqual(transformOne, transformTwo);
         numberTestsComplete++;
      }

      if (VERBOSE)
      {
         System.out.println("numberTestsComplete = " + numberTestsComplete);
         System.out.println("nextTransformToRootID = " + ReferenceFrame.nextTransformToRootID);
      }
      tearDown();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testUpdateInMiddleFrame()
   {
      TranslationReferenceFrame frame1 = new TranslationReferenceFrame("frame1", ReferenceFrame.getWorldFrame());
      TranslationReferenceFrame frame2 = new TranslationReferenceFrame("frame2", frame1);
      TranslationReferenceFrame frame3 = new TranslationReferenceFrame("frame3", frame2);

      Vector3d translation1 = new Vector3d(0.1, 0.13, 0.45);
      Vector3d translation2 = new Vector3d(0.7, 0.26, 0.09);
      Vector3d translation3 = new Vector3d(0.04, 0.023, 0.067);

      frame1.updateTranslation(translation1);
      frame2.updateTranslation(translation2);
      frame3.updateTranslation(translation3);

      RigidBodyTransform transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      Vector3d totalTranslation = new Vector3d();

      Vector3d expectedTranslation = new Vector3d(translation1);
      expectedTranslation.add(translation2);
      expectedTranslation.add(translation3);

      transformToDesiredFrame.getTranslation(totalTranslation);

      JUnitTools.assertTuple3dEquals(expectedTranslation, totalTranslation, 1e-7);

      translation2.set(0.33, 0.44, 0.11);
      frame2.updateTranslation(translation2);

      transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      totalTranslation = new Vector3d();

      expectedTranslation = new Vector3d(translation1);
      expectedTranslation.add(translation2);
      expectedTranslation.add(translation3);

      transformToDesiredFrame.getTranslation(totalTranslation);

      JUnitTools.assertTuple3dEquals(expectedTranslation, totalTranslation, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testUpdateInMiddleWithCorruptors()
   {
      TransformReferenceFrame frame1 = new TransformReferenceFrame("frame1", ReferenceFrame.getWorldFrame());
      TransformReferenceFrame frame2 = new TransformReferenceFrame("frame2", frame1);
      TransformReferenceFrame frame3 = new TransformReferenceFrame("frame3", frame2);

      RigidBodyTransform transform1 = new RigidBodyTransform();
      transform1.setRotationRollAndZeroTranslation(0.77);
      transform1.setTranslation(new Vector3d(0.1, 0.13, 0.45));

      RigidBodyTransform transform2 = new RigidBodyTransform();
      transform2.setRotationPitchAndZeroTranslation(0.4);
      transform2.setTranslation(new Vector3d(0.5, 0.12, 0.35));

      RigidBodyTransform transform3 = new RigidBodyTransform();
      transform3.setRotationYawAndZeroTranslation(0.37);
      transform3.setTranslation(new Vector3d(0.11, 0.113, 0.415));

      frame1.setTransformAndUpdate(transform1);
      frame2.setTransformAndUpdate(transform2);
      frame3.setTransformAndUpdate(transform3);

      RigidBodyTransform transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      RigidBodyTransform expectedTransform = new RigidBodyTransform(transform1);
      expectedTransform.multiply(transform2);
      expectedTransform.multiply(transform3);

      RigidBodyTransformTest.assertTransformEquals(expectedTransform, transformToDesiredFrame, 1e-7);

      RigidBodyTransform preCorruptionTransform = new RigidBodyTransform();
      preCorruptionTransform.setRotationRollAndZeroTranslation(0.35);
      preCorruptionTransform.setTranslation(new Vector3d(0.51, 0.113, 0.7415));
      frame2.corruptTransformToParentPreMultiply(preCorruptionTransform);

      RigidBodyTransform postCorruptionTransform = new RigidBodyTransform();
      postCorruptionTransform.setRotationPitchAndZeroTranslation(0.97);
      postCorruptionTransform.setTranslation(new Vector3d(0.12, 0.613, 0.415));
      frame2.corruptTransformToParentPostMultiply(postCorruptionTransform);

      expectedTransform = new RigidBodyTransform(transform1);
      expectedTransform.multiply(preCorruptionTransform);
      expectedTransform.multiply(transform2);
      expectedTransform.multiply(postCorruptionTransform);
      expectedTransform.multiply(transform3);

      transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      RigidBodyTransformTest.assertTransformEquals(expectedTransform, transformToDesiredFrame, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testGetTransformBetweenFramesTwo()
   {
      setUp();
      Random random = new Random(1776L);
      updateAllFrames();

      int numberOfTests = 100000;
      int numberTestsComplete = 0;

      for (int i = 0; i < numberOfTests; i++)
      {
         ReferenceFrame frame1 = selectARandomFrame(random);
         ReferenceFrame frame2 = selectARandomFrame(random);

         updateARandomFrame(random);

         if (frame1.getRootFrame() != frame2.getRootFrame())
         {
            continue;
         }

         RigidBodyTransform transformOne = frame1.getTransformToDesiredFrame(frame2);
         RigidBodyTransform transformTwo = getTransformToDesiredFrameThroughVerificationTransforms(frame1, frame2);

         verifyTransformsAreEpsilonEqual(transformOne, transformTwo);
         numberTestsComplete++;
      }

      if (VERBOSE)
      {
         System.out.println("numberTestsComplete = " + numberTestsComplete);
         System.out.println("nextTransformToRootID = " + ReferenceFrame.nextTransformToRootID);
      }
      tearDown();
   }
	
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testConstructFrameFromPointAndAxis()
   {
      setUp();
      Random random = new Random(1776L);
      updateAllFrames();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePoint randomPoint = new FramePoint(worldFrame);

      FrameVector randomVector = new FrameVector(worldFrame);

      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         randomPoint.setIncludingFrame(FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0));
         randomVector.setIncludingFrame(FrameVector.generateRandomFrameVector(random, worldFrame, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0));

         ReferenceFrame frameA = ReferenceFrame.constructReferenceFrameFromPointAndZAxis("frameA", randomPoint, randomVector);
         ReferenceFrame frameB = ReferenceFrame.constructReferenceFrameFromPointAndAxis("frameB", randomPoint, Axis.Z, randomVector);

         verifyTransformsAreEpsilonEqual(frameA.getTransformToWorldFrame(), frameB.getTransformToWorldFrame());
      }
      tearDown();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testCopyAndAlignAxisWithVector()
   {
      setUp();
      Random random = new Random(1776L);
      updateAllFrames();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      RigidBodyTransform transformA = new RigidBodyTransform();
      RigidBodyTransform transformB = new RigidBodyTransform();
      
      FrameVector zAxisFrameA = new FrameVector();
      FrameVector zAxisFrameB = new FrameVector();
      
      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         transformA.setRotationAndZeroTranslation(RandomTools.generateRandomQuaternion(random, Math.toRadians(80.0)));
         transformB.setRotationAndZeroTranslation(RandomTools.generateRandomQuaternion(random, Math.toRadians(80.0)));

         ReferenceFrame frameA = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frameA", worldFrame, transformA);
         ReferenceFrame frameB = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frameB", worldFrame, transformB);

         frameB = frameB.copyAndAlignAxisWithVector(Axis.Z, new FrameVector(frameA, 0.0, 0.0, 1.0));
         
         zAxisFrameA.setIncludingFrame(frameA, 0.0, 0.0, 1.0);
         zAxisFrameB.setIncludingFrame(frameB, 0.0, 0.0, 1.0);
         zAxisFrameB.changeFrame(frameA);

         assertTrue(zAxisFrameA.isEpsilonParallel(zAxisFrameB, Math.toRadians(0.1)));
         assertEquals("zAxis in FrameB is not aligned with zAxis in FrameA!", 1.0, zAxisFrameA.dot(zAxisFrameB) / (zAxisFrameA.length() * zAxisFrameB.length()), 1e-7);
      }
      tearDown();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testCopyAndAlignTwoAxesWithTwoVectors()
   {
      setUp();
      Random random = new Random(1776L);
      updateAllFrames();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      RigidBodyTransform transformA = new RigidBodyTransform();
      RigidBodyTransform transformB = new RigidBodyTransform();
      
      FrameVector xAxisFrameA = new FrameVector();
      FrameVector xAxisFrameB = new FrameVector();
      FrameVector zAxisFrameA = new FrameVector();
      FrameVector zAxisFrameB = new FrameVector();
      
      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         transformA.setRotationAndZeroTranslation(RandomTools.generateRandomQuaternion(random, Math.toRadians(80.0)));
         transformB.setRotationAndZeroTranslation(RandomTools.generateRandomQuaternion(random, Math.toRadians(80.0)));

         ReferenceFrame frameA = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frameA", worldFrame, transformA);
         ReferenceFrame frameB = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frameB", worldFrame, transformB);

         frameB = frameB.copyAndAlignAxisWithVector(Axis.Z, new FrameVector(frameA, 0.0, 0.0, 1.0));
         frameB = frameB.copyAndAlignAxisWithVector(Axis.X, new FrameVector(frameA, 1.0, 0.0, 0.0));
                  
         xAxisFrameA.setIncludingFrame(frameA, 1.0, 0.0, 0.0);
         xAxisFrameB.setIncludingFrame(frameB, 1.0, 0.0, 0.0);
         xAxisFrameB.changeFrame(frameA);

         assertTrue(xAxisFrameA.isEpsilonParallel(xAxisFrameB, Math.toRadians(0.1)));
         
         zAxisFrameA.setIncludingFrame(frameA, 0.0, 0.0, 1.0);
         zAxisFrameB.setIncludingFrame(frameB, 0.0, 0.0, 1.0);
         zAxisFrameB.changeFrame(frameA);

         assertTrue(zAxisFrameA.isEpsilonParallel(zAxisFrameB, Math.toRadians(0.1)));
      }
      tearDown();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testCopyAndAimAxisAtPoint()
   {
      setUp();
      Random random = new Random(1776L);
      updateAllFrames();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      RigidBodyTransform transformA = new RigidBodyTransform();
      
      Point3d frameOriginInWorld = new Point3d();
      
      FrameVector xAxis = new FrameVector(worldFrame, 1.0, 0.0, 0.0);
      Vector3d xAxisInWorld = new Vector3d(1.0, 0.0, 0.0);
      FramePoint aimAxisAtThis = new FramePoint();
      
      FrameVector axisToAlign = new FrameVector();
      
      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         transformA.set(RigidBodyTransform.generateRandomTransform(random));
         ReferenceFrame frameA = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frameA", worldFrame, transformA);

         transformA.getTranslation(frameOriginInWorld);
         aimAxisAtThis.setIncludingFrame(worldFrame, frameOriginInWorld);
         aimAxisAtThis.add(xAxisInWorld);

         ReferenceFrame frameB = frameA.copyAndAimAxisAtPoint(Axis.X, aimAxisAtThis);
         
         axisToAlign.setIncludingFrame(frameB, 1.0, 0.0, 0.0);
         axisToAlign.changeFrame(worldFrame);

         assertTrue(xAxis.isEpsilonParallel(axisToAlign, Math.toRadians(0.1)));
      }
      tearDown();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testWorldFrameSerializable()
   {
      setUp();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Assertions.assertSerializable(worldFrame);
      tearDown();

      //NOTE:No other reference frame is serializable because of transform3D
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.7)
	@Test(timeout = 30000)
   public void testGarbageCollectionInBroadTrees()
   {
      int beforeMemoryInMB = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("ReferenceFrameTest: before");

      ReferenceFrame world = ReferenceFrame.getWorldFrame();

      List<ReferenceFrame> testFrames = new ArrayList<ReferenceFrame>();
      for (int i = 0; i < 100000; i++)
      {
         PoseReferenceFrame testFrame = new PoseReferenceFrame("test_" + i, world);
         testFrames.add(testFrame);
      }
      
      try
      {
         Thread.sleep(100);
      }
      catch (InterruptedException e)
      {
      }

      int duringMemoryInMB = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMBWithoutGarbageCollecting("ReferenceFrameTest: during");

      assertTrue(duringMemoryInMB - beforeMemoryInMB > 20);

      int afterMemoryInMB = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("ReferenceFrameTest: after");
      assertTrue("afterMemoryInMB - beforeMemoryInMB = " + (afterMemoryInMB - beforeMemoryInMB), afterMemoryInMB - beforeMemoryInMB < 140);

      testFrames = null;
      afterMemoryInMB = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("ReferenceFrameTest: after testFrames = null");
      assertTrue(afterMemoryInMB - beforeMemoryInMB < 10);

   }

   private void updateARandomFrame(Random random)
   {
      ReferenceFrame frame = selectARandomFrame(random);
      frame.update();
   }

   private ReferenceFrame selectARandomFrame(Random random)
   {
      int index = random.nextInt(allFramesTogether.size());

      return allFramesTogether.get(index);
   }

   private void verifyTransformToRootByClimbingTree(ReferenceFrame frame, RigidBodyTransform transformToRootOne)
   {
      RigidBodyTransform transformToRootTwo = getTransformToDesiredAncestorByClimbingTree(frame, null);
      RigidBodyTransform transformToRootThree = getTransformToDesiredAncestorThroughVerificationTransforms(frame, null);
      verifyTransformsAreEpsilonEqual(transformToRootOne, transformToRootTwo, transformToRootThree);
   }

   private RigidBodyTransform getTransformToDesiredAncestorByClimbingTree(ReferenceFrame frame, ReferenceFrame desiredFrame)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      ReferenceFrame nextFrame = frame;
      while (true)
      {
         ReferenceFrame parent = nextFrame.getParent();
         if (parent == null)
         {
            break;
         }

         RigidBodyTransform transformToParent = nextFrame.getTransformToParent();
         RigidBodyTransform transform = new RigidBodyTransform(transformToParent);
         transform.multiply(ret);

         ret.set(transform);

         nextFrame = parent;
         if (nextFrame == desiredFrame)
         {
            break;
         }
      }

      return ret;
   }

   private RigidBodyTransform getTransformToDesiredAncestorThroughVerificationTransforms(ReferenceFrame frame, ReferenceFrame desiredFrame)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      ReferenceFrame nextFrame = frame;
      while (true)
      {
         ReferenceFrame parent = nextFrame.getParent();
         if (parent == null)
         {
            break;
         }

         RigidBodyTransform transformToParent = new RigidBodyTransform(transformsForVerification.get(nextFrame.getName()));
         RigidBodyTransform transform = new RigidBodyTransform(transformToParent);
         transform.multiply(ret);

         ret.set(transform);

         nextFrame = parent;
         if (nextFrame == desiredFrame)
         {
            break;
         }
      }

      return ret;
   }

   private RigidBodyTransform getTransformToDesiredFrameThroughVerificationTransforms(ReferenceFrame frame, ReferenceFrame desiredFrame)
   {
      RigidBodyTransform transformOne = getTransformToDesiredAncestorThroughVerificationTransforms(frame, null);
      RigidBodyTransform transformTwo = getTransformToDesiredAncestorThroughVerificationTransforms(desiredFrame, null);
      transformTwo.invert();
      transformTwo.multiply(transformOne);

      return transformTwo;
   }

   private void verifyTransformsAreEpsilonEqual(RigidBodyTransform transformOne, RigidBodyTransform transformTwo)
   {
      if (!epsilonEquals(transformOne, transformTwo, 1e-2))
      {  
         System.err.println("transformOne = " + transformOne);
         System.err.println("transformTwo = " + transformTwo);
         fail();
      }
   }

   private void verifyTransformsAreEpsilonEqual(RigidBodyTransform transformToParentOne, RigidBodyTransform transformToParentTwo, RigidBodyTransform transformToParentThree)
   {
      if (!epsilonEquals(transformToParentOne, transformToParentTwo, 0.001))
      {
         fail();
      }
      if (!epsilonEquals(transformToParentTwo, transformToParentThree, 0.001))
      {
         fail();
      }
   }

   private boolean epsilonEquals(RigidBodyTransform transformOne, RigidBodyTransform transformTwo, double epsilonPercent)
   {
      double maxDeltaPercent = getMaxDeltaPercent(transformOne, transformTwo);

      return (maxDeltaPercent < epsilonPercent);
   }

   private double getMaxDeltaPercent(RigidBodyTransform t1, RigidBodyTransform t2)
   {
      double[] arg1 = new double[16];
      double[] arg2 = new double[16];

      t1.get(arg1);
      t2.get(arg2);

      return getMaxDeltaPercent(arg1, arg2);
   }

   private double getMaxDeltaPercent(double[] arg1, double[] arg2)
   {
      double max = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < arg1.length; i++)
      {
         double absolute1 = Math.abs(arg1[i]);
         double absolute2 = Math.abs(arg2[i]);

         if ((absolute1 < 1e-7) && (absolute2 < 1e-7))
         {
            if (max < 0.0)
            {
               max = 0.0;
            }
         }

         else
         {
            double absoluteDifference = Math.abs(arg1[i] - arg2[i]);
            double largestOne = Math.max(Math.abs(arg1[i]), Math.abs(arg2[i]));
            double percentDifference = absoluteDifference / largestOne;

            if (percentDifference > max)
            {
               max = percentDifference;
            }
         }
      }

      return max;
   }

   private void updateAllFrames()
   {
      for (ReferenceFrame frame : allFramesTogether)
      {
         frame.update();
      }
   }

}
