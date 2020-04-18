package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.function.Function;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.robotics.physics.EuclidFrameShapeCollisionTools.FrameShape3DCollisionEvaluator;
import us.ihmc.robotics.physics.EuclidFrameShapeCollisionTools.Shape3DCollisionEvaluator;

class EuclidFrameShapeCollisionToolsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;
   private static final double EPA_EPSILON = 2.0e-4;

   private <A extends Shape3DReadOnly, B extends Shape3DReadOnly> void performAssertionsForCollisionEvaluator(Random random,
                                                                                                              FrameShape3DCollisionEvaluator<A, B> evaluatorToTest,
                                                                                                              Shape3DCollisionEvaluator<A, B> referenceForTesting,
                                                                                                              Function<Random, A> shapeARandomGenerator,
                                                                                                              Function<Random, B> shapeBRandomGenerator,
                                                                                                              EuclidFrameShapeTools.FrameChanger<A> shapeAFrameChanger,
                                                                                                              EuclidFrameShapeTools.FrameChanger<B> shapeBFrameChanger,
                                                                                                              boolean testNormal, double epsilon)
   {
      for (int i = 0; i < ITERATIONS; i++)
      { // Test in world frame
         EuclidFrameShape3DCollisionResult actual = new EuclidFrameShape3DCollisionResult();
         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         A shapeA = shapeARandomGenerator.apply(random);
         ReferenceFrame frameA = worldFrame;
         B shapeB = shapeBRandomGenerator.apply(random);
         ReferenceFrame frameB = worldFrame;
         evaluatorToTest.evaluateCollision(shapeA, frameA, shapeB, frameB, actual);
         referenceForTesting.evaluateCollision(shapeA, shapeB, expected);
         assertCollisionResultsEqual(expected, frameA, frameB, actual, testNormal, epsilon);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test in a random frame (frameA == frameB)
         EuclidFrameShape3DCollisionResult actual = new EuclidFrameShape3DCollisionResult();
         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         A shapeA = shapeARandomGenerator.apply(random);
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);
         B shapeB = shapeBRandomGenerator.apply(random);
         ReferenceFrame frameB = frameA;
         evaluatorToTest.evaluateCollision(shapeA, frameA, shapeB, frameB, actual);
         referenceForTesting.evaluateCollision(shapeAFrameChanger.changeFrame(shapeA, frameA, worldFrame),
                                               shapeBFrameChanger.changeFrame(shapeB, frameB, worldFrame),
                                               expected);
         expected.setShapeA(shapeA);
         expected.setShapeB(shapeB);
         actual.getPointOnA().changeFrame(worldFrame);
         actual.getPointOnB().changeFrame(worldFrame);
         actual.getNormalOnA().changeFrame(worldFrame);
         actual.getNormalOnB().changeFrame(worldFrame);
         assertCollisionResultsEqual(expected, frameA, frameB, actual, testNormal, epsilon);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test in random frames (frameA != frameB)
         EuclidFrameShape3DCollisionResult actual = new EuclidFrameShape3DCollisionResult();
         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         A shapeA = shapeARandomGenerator.apply(random);
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         B shapeB = shapeBRandomGenerator.apply(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         evaluatorToTest.evaluateCollision(shapeA, frameA, shapeB, frameB, actual);
         referenceForTesting.evaluateCollision(shapeAFrameChanger.changeFrame(shapeA, frameA, worldFrame),
                                               shapeBFrameChanger.changeFrame(shapeB, frameB, worldFrame),
                                               expected);
         expected.setShapeA(shapeA);
         expected.setShapeB(shapeB);
         actual.getPointOnA().changeFrame(worldFrame);
         actual.getPointOnB().changeFrame(worldFrame);
         actual.getNormalOnA().changeFrame(worldFrame);
         actual.getNormalOnB().changeFrame(worldFrame);
         assertCollisionResultsEqual(expected, frameA, frameB, actual, testNormal, epsilon);
      }
   }

   private static void assertCollisionResultsEqual(EuclidShape3DCollisionResult expected, ReferenceFrame expectedFrameA, ReferenceFrame expectedFrameB,
                                                   EuclidFrameShape3DCollisionResult actual, boolean testNormal, double epsilon)
   {
      assertTrue(actual.getFrameA() == expectedFrameA);
      assertTrue(actual.getFrameB() == expectedFrameB);

      if (!testNormal)
      {
         expected.getNormalOnA().setToNaN();
         expected.getNormalOnB().setToNaN();
      }
      else
      {
         expected.getNormalOnA().normalize();
         expected.getNormalOnB().normalize();
         actual.getNormalOnA().normalize();
         actual.getNormalOnB().normalize();
      }

      EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, actual, epsilon);
   }

   @Test
   public void testEvaluateCapsule3DCapsule3DCollision()
   {
      Random random = new Random(345);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateCapsule3DCapsule3DCollision,
                                             EuclidFrameShapeCollisionTools.capsule3DToCapsule3DEvaluator,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DCapsule3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.capsule3DToCapsule3DEvaluator,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.capsule3DToCapsule3DEvaluator,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DBox3DCollision()
   {
      Random random = new Random(5165);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DBox3DCollision,
                                             EuclidFrameShapeCollisionTools.pointShape3DToBox3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextBox3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DBox3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToBox3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextBox3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToBox3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextBox3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DCapsule3DCollision()
   {
      Random random = new Random(54687);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DCapsule3DCollision,
                                             EuclidFrameShapeCollisionTools.pointShape3DToCapsule3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DCapsule3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToCapsule3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToCapsule3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DCylinder3DCollision()
   {
      Random random = new Random(86);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DCylinder3DCollision,
                                             EuclidFrameShapeCollisionTools.pointShape3DToCylinder3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextCylinder3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DCylinder3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToCylinder3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextCylinder3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToCylinder3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextCylinder3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DEllipsoid3DCollision()
   {
      Random random = new Random(285);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DEllipsoid3DCollision,
                                             EuclidFrameShapeCollisionTools.pointShape3DToEllipsoid3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextEllipsoid3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DEllipsoid3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToEllipsoid3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextEllipsoid3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToEllipsoid3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextEllipsoid3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DPointShape3DCollision()
   {
      Random random = new Random(6);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DPointShape3DCollision,
                                             EuclidFrameShapeCollisionTools.pointShape3DToPointShape3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DPointShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToPointShape3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToPointShape3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DRamp3DCollision()
   {
      Random random = new Random(98587621);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DRamp3DCollision,
                                             EuclidFrameShapeCollisionTools.pointShape3DToRamp3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextRamp3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DRamp3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToRamp3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextRamp3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToRamp3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextRamp3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DSphere3DCollision()
   {
      Random random = new Random(74275);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DSphere3DCollision,
                                             EuclidFrameShapeCollisionTools.pointShape3DToSphere3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DSphere3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToSphere3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);

      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateShape3DShape3DCollisionEPA,
                                             EuclidFrameShapeCollisionTools.pointShape3DToSphere3DEvaluator,
                                             EuclidShapeRandomTools::nextPointShape3D,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             false,
                                             EPA_EPSILON);
   }

   @Test
   public void testEvaluateSphere3DBox3DCollision()
   {
      Random random = new Random(5165);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DBox3DCollision,
                                             EuclidFrameShapeCollisionTools.sphere3DToBox3DEvaluator,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidShapeRandomTools::nextBox3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DCapsule3DCollision()
   {
      Random random = new Random(54687);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DCapsule3DCollision,
                                             EuclidFrameShapeCollisionTools.sphere3DToCapsule3DEvaluator,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidShapeRandomTools::nextCapsule3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DCylinder3DCollision()
   {
      Random random = new Random(86);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DCylinder3DCollision,
                                             EuclidFrameShapeCollisionTools.sphere3DToCylinder3DEvaluator,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidShapeRandomTools::nextCylinder3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DEllipsoid3DCollision()
   {
      Random random = new Random(285);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DEllipsoid3DCollision,
                                             EuclidFrameShapeCollisionTools.sphere3DToEllipsoid3DEvaluator,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidShapeRandomTools::nextEllipsoid3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DRamp3DCollision()
   {
      Random random = new Random(98587621);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DRamp3DCollision,
                                             EuclidFrameShapeCollisionTools.sphere3DToRamp3DEvaluator,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidShapeRandomTools::nextRamp3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DSphere3DCollision()
   {
      Random random = new Random(74275);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DSphere3DCollision,
                                             EuclidFrameShapeCollisionTools.sphere3DToSphere3DEvaluator,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidShapeRandomTools::nextSphere3D,
                                             EuclidFrameShapeTools::changeFrame,
                                             EuclidFrameShapeTools::changeFrame,
                                             true,
                                             EPSILON);
   }

}
