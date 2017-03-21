package us.ihmc.simulationconstructionset.util.ground;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class RotatableRampTerrainObjectTest
{
   private RotatableRampTerrainObject simpleRamp, simpleRampDown, ramp90;
   private RotatableRampTerrainObject simpleRampTranslated, ramp90Translated;
   private double epsilon = 1e-12;
   
   private Point3D pointsOnSimpleRamp[] =
   {
      new Point3D(0, 0, 0), new Point3D(1, 0, 1), new Point3D(0.5, 0, 0.5), new Point3D(0.5, -1, 0.5), new Point3D(0.5, 1, 0.5), new Point3D(1, 1, 1),
      new Point3D(1,-1,1)
   };
   
   private Point3D strictlyInternalPointsOnSimpleRampDown[] =
   {
      new Point3D(0.001, 0.0, 0.999), new Point3D(0.999, 0, 0.001), new Point3D(0.5, 0, 0.5), 
      new Point3D(0.5, -0.999, 0.5),  new Point3D(0.5, 0.999, 0.5), 
      new Point3D(0.999, 0.999, 0.001)
   };
   
   private Point3D pointsOnOtherRampFaces[] = {new Point3D(1, 0, 0.5), new Point3D(0.5, 1, 0.25), new Point3D(0.5, -1, 0.25)};
   private Vector3D expectedSimpleSurfaceNormal = new Vector3D(-1, 0, 1);
   private Vector3D expectedSimpleSurfaceNormalOnOtherFaces[] = {new Vector3D(1, 0, 0), new Vector3D(0, 1, 0), new Vector3D(0, -1, 0)};
   private Point3D pointsOnOtherRampFacesSlopeDown[] = {new Point3D(0, 0, 0.5), new Point3D(0.5, 1, 0.25), new Point3D(0.5, -1, 0.25)};
   private Vector3D expectedSimpleSurfaceNormalSlopeDown = new Vector3D(1, 0, 1);
   private Vector3D expectedSimpleSurfaceNormalOnOtherFacesSlopeDown[] = {new Vector3D(-1, 0, 0), new Vector3D(0, 1, 0), new Vector3D(0, -1, 0)};

   private Point3D pointsOnRamp90[] =
   {
      new Point3D(0, 0, 0.5), new Point3D(1, 0, 0.5), new Point3D(-1, 0, 0.5),
      new Point3D(0, -0.49, 0.01),  new Point3D(1, -0.5, 0), new Point3D(-0.99, -0.499, 0.001),
      new Point3D(0.5, 0.25, 0.75), new Point3D(0.9, 0.4, 0.9), new Point3D(1.0, 0.4, 0.9), new Point3D(1.0, 0.45, 0.95), new Point3D(1.0, 0.499, 0.999),
      new Point3D(0, 0.5, 1), new Point3D(-1, 0.5, 1), new Point3D(0.9, 0.5,1)//, new Point3D(0.909, 0.5,1),//, new Point3D(1, 0.5, 1)
      
   };
   
   private Point3D pointsOnRamp90Translated[] =
   {
      new Point3D(0, 0, 0.5), new Point3D(-0.99, -0.499, 0.001),
      new Point3D(0.9, 0.4, 0.9)      
   };
   
   private Point3D pointsOnRamp90PassingHeightCornerCases[] =
   {
      new Point3D(-1, -0.5, 0), new Point3D(0, -0.5, 0)
      
   };
   private Point3D pointsOnRamp90withNumericalRotationError[] =
   {
         new Point3D(0.909, 0.5,1), new Point3D(1, 0.5, 1)
      
   };
   private Vector3D expectedSurfaceNormalRamp90 = new Vector3D(0, -1, 1);
   private Point3D pointsOnOtherFacesRamp90[] = {new Point3D(0, 0.5, 0.5), new Point3D(1, 0, 0.25), new Point3D(-1, 0, 0.25)};
   private Vector3D expectedSurfaceNormalOnOtherFacesRamp90[] = {new Vector3D(0, 1, 0), new Vector3D(1, 0, 0), new Vector3D(-1, 0, 0)};
   
   private static double transX=3.0;
   private static double transY=2.0;
   
   @Before
   public void setUp() throws Exception
   {
      simpleRamp = new RotatableRampTerrainObject(0.5, 0, 1, 2, 1, 0);
      simpleRampDown = new RotatableRampTerrainObject(0.5, 0, -1, 2, 1, 0);
      ramp90 = new RotatableRampTerrainObject(0, 0, 1, 2, 1, 90);
      simpleRampTranslated = new RotatableRampTerrainObject(transX+0.5, transY, 1, 2, 1, 0);
      ramp90Translated = new RotatableRampTerrainObject(transX, transY, 1, 2, 1, 90);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHeightAt()
   {
      testHeightAtRampForAnyRamp(pointsOnSimpleRamp, simpleRamp);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHeightAtForRampDown()
   {
      testHeightAtRampForAnyRamp(strictlyInternalPointsOnSimpleRampDown, simpleRampDown);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSurfaceNormalAt()
   {
      testSurfaceNormalsForAnyRampFace(simpleRamp, expectedSimpleSurfaceNormal, pointsOnSimpleRamp);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testOtherSurfaceNormalAt()
   {
      testSurfaceNormalsForAnyOtherRampSides(simpleRamp, 
            expectedSimpleSurfaceNormalOnOtherFaces, pointsOnOtherRampFaces);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSurfaceNormalAtForSlopedDown()
   {
      testSurfaceNormalsForAnyRampFace(simpleRampDown, 
            expectedSimpleSurfaceNormalSlopeDown, strictlyInternalPointsOnSimpleRampDown);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testOtherSurfaceNormalAtForSlopedDown()
   {
      testSurfaceNormalsForAnyOtherRampSides(simpleRampDown,
            expectedSimpleSurfaceNormalOnOtherFacesSlopeDown, pointsOnOtherRampFacesSlopeDown);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHeightAtRamp90()
   {
      testHeightAtRampForAnyRamp(pointsOnRamp90, ramp90);
      testHeightAtRampForAnyRamp(pointsOnRamp90PassingHeightCornerCases, ramp90);      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void HeightAtRamp90EdgeCasesFailDueToNumericalErrorTest()
   {
      testHeightAtRampForAnyRamp(pointsOnRamp90withNumericalRotationError, ramp90);
   }   

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)   
   public void testSurfaceNormalForRamp90()
   {
      testSurfaceNormalsForAnyRampFace(ramp90, 
            expectedSurfaceNormalRamp90, pointsOnRamp90);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)   
   public void testOtherSurfaceNormalForRamp90()
   {
      testSurfaceNormalsForAnyOtherRampSides(ramp90, 
            expectedSurfaceNormalOnOtherFacesRamp90, pointsOnOtherFacesRamp90);
   }

   private void testHeightAtRampForAnyRamp(Point3D[] pointsOnRamp, RotatableRampTerrainObject ramp)
   {
      for (int i = 0; i < pointsOnRamp.length; i++)
      {
         String message = "Expected Height For point " + pointsOnRamp[i].getX() + " " + pointsOnRamp[i].getY() + " " + pointsOnRamp[i].getZ();
         assertEquals(message, pointsOnRamp[i].getZ(), ramp.heightAt(pointsOnRamp[i].getX(), pointsOnRamp[i].getY(), pointsOnRamp[i].getZ()), epsilon);
      }
   }

   private void testHeightAtRampForAnyRampWithTranslation(Point3D[] pointsOnRamp, RotatableRampTerrainObject ramp, Vector3D translation)
   {
      for (int i = 0; i < pointsOnRamp.length; i++)
      {
         String message = "Expected Height For point " + (pointsOnRamp[i].getX()+translation.getX()) + " " + 
               (pointsOnRamp[i].getY()+translation.getY()) + " " + pointsOnRamp[i].getZ();
         assertEquals(message, pointsOnRamp[i].getZ(), ramp.heightAt(pointsOnRamp[i].getX()+translation.getX(), pointsOnRamp[i].getY()+translation.getY(), pointsOnRamp[i].getZ()), epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHeightAtTranslation()
   {
      testHeightAtRampForAnyRampWithTranslation(pointsOnSimpleRamp, simpleRampTranslated, new Vector3D(transX,transY,0));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHeightAt90Translation()
   {
      testHeightAtRampForAnyRampWithTranslation(pointsOnRamp90Translated, ramp90Translated, new Vector3D(transX,transY,0));
   }


   private void testSurfaceNormalsForAnyRampFace(RotatableRampTerrainObject ramp, 
         Vector3D expectedRampSurfaceNormal, Point3D[] pointsOnRamp)
 {
    expectedRampSurfaceNormal.normalize();

    for (int i = 0; i < pointsOnRamp.length; i++)
    {
       Vector3D normal = new Vector3D();
       ramp.surfaceNormalAt(pointsOnRamp[i].getX(), pointsOnRamp[i].getY(), pointsOnRamp[i].getZ(), normal);
       String message = "Normal for point " + pointsOnRamp[i].getX() + " " + pointsOnRamp[i].getY() + " " + pointsOnRamp[i].getZ();
       EuclidCoreTestTools.assertTuple3DEquals(message, expectedRampSurfaceNormal, normal, epsilon);
    }
 }
 
   private void testSurfaceNormalsForAnyOtherRampSides(RotatableRampTerrainObject ramp,
         Vector3D[] expectedSurfaceNormalOnOtherFaces, Point3D[] pointsOnOtherRampFaces)
 {
    for (int i = 0; i < pointsOnOtherRampFaces.length; i++)
    {
       expectedSurfaceNormalOnOtherFaces[i].normalize();
       Vector3D normal = new Vector3D();
       ramp.surfaceNormalAt(pointsOnOtherRampFaces[i].getX(), pointsOnOtherRampFaces[i].getY(), pointsOnOtherRampFaces[i].getZ(), normal);
       String message = "Normal for point " + pointsOnOtherRampFaces[i].getX() + " " + pointsOnOtherRampFaces[i].getY() + " "
                        + pointsOnOtherRampFaces[i].getZ();
       EuclidCoreTestTools.assertTuple3DEquals(message, expectedSurfaceNormalOnOtherFaces[i], normal, epsilon);
    }
 }
 
   


}
