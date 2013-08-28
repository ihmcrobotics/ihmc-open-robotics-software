package com.yobotics.simulationconstructionset.util.ground;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.Plane3d;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class RotatableRampTerrainObjectTest
{
   private RotatableRampTerrainObject simpleRamp, simpleRampDown, ramp90;
   private double epsilon = 1e-12;
   private Point3d pointsOnSimpleRamp[] =
   {
      new Point3d(0, 0, 0), new Point3d(1, 0, 1), new Point3d(0.5, 0, 0.5), new Point3d(0.5, -1, 0.5), new Point3d(0.5, 1, 0.5), new Point3d(1, 1, 1)
   };
   private Point3d pointsOnSimpleRampDown[] =
   {
      new Point3d(0, 0, 1), new Point3d(1, 0, 0), new Point3d(0.5, 0, 0.5), new Point3d(0.5, -1, 0.5), new Point3d(0.5, 1, 0.5), new Point3d(1, 1, 0)
   };
   private Point3d pointsOnOtherRampFaces[] = {new Point3d(1, 0, 0.5), new Point3d(0.5, 1, 0.25), new Point3d(0.5, -1, 0.25)};
   private Vector3d expectedSimpleSurfaceNormal = new Vector3d(-1, 0, 1);
   private Vector3d expectedSimpleSurfaceNormalOnOtherFaces[] = {new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, -1, 0)};
   private Point3d pointsOnOtherRampFacesSlopeDown[] = {new Point3d(0, 0, 0.5), new Point3d(0.5, 1, 0.25), new Point3d(0.5, -1, 0.25)};
   private Vector3d expectedSimpleSurfaceNormalSlopeDown = new Vector3d(1, 0, 1);
   private Vector3d expectedSimpleSurfaceNormalOnOtherFacesSlopeDown[] = {new Vector3d(-1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, -1, 0)};

   private Point3d pointsOnRamp90[] =
   {
      new Point3d(0, 0, 0.5), new Point3d(1, 0, 0.5), new Point3d(-1, 0, 0.5), new Point3d(0, -0.5, 0), new Point3d(1, -0.5, 0), new Point3d(-1, -0.5, 0),
      new Point3d(0, 0.5, 1), new Point3d(1, 0.5, 1), new Point3d(-1, 0.5, 1)
   };
   private Vector3d expectedSurfaceNormalRamp90 = new Vector3d(0, -1, 1);
   private Point3d pointsOnOtherFacesRamp90[] = {new Point3d(0, 0.5, 0.5), new Point3d(1, 0, 0.25), new Point3d(-1, 0, 0.25)};
   private Vector3d expectedSurfaceNormalOnOtherFacesRamp90[] = {new Vector3d(0, 1, 0), new Vector3d(1, 0, 0), new Vector3d(-1, 0, 0)};


   @Before
   public void setUp() throws Exception
   {
      simpleRamp = new RotatableRampTerrainObject(0, -1, 1, 1, 1, 0);
      simpleRampDown = new RotatableRampTerrainObject(1, -1, 0, 1, 1, 0);
      ramp90 = new RotatableRampTerrainObject(-0.5, -1, 0.5, 1, 1, 90);
   }

   @Test
   public void testHeightAt()
   {
      testHeightAtRampForAnyRamp(pointsOnSimpleRamp, simpleRamp);
   }

   @Test
   public void testHeightAtForRampDown()
   {
      testHeightAtRampForAnyRamp(pointsOnSimpleRampDown, simpleRampDown);
   }

   @Test
   public void testSurfaceNormalAt()
   {
      testSurfaceNormalsForAnyRamp(simpleRamp, expectedSimpleSurfaceNormal, pointsOnSimpleRamp, expectedSimpleSurfaceNormalOnOtherFaces,
                                   pointsOnOtherRampFaces);
   }

   @Test
   public void testSurfaceNormalAtForSlopedDown()
   {
      testSurfaceNormalsForAnyRamp(simpleRampDown, expectedSimpleSurfaceNormalSlopeDown, pointsOnSimpleRampDown,
                                   expectedSimpleSurfaceNormalOnOtherFacesSlopeDown, pointsOnOtherRampFacesSlopeDown);
   }

   /**
    *
    */
   @Test @Ignore//working to get this one to pass
   public void testHeightAtRamp90()
   {
      testHeightAtRampForAnyRamp(pointsOnRamp90, ramp90);
   }

   @Test   @Ignore//Not implemented yet... Until previous error fixed, don't worry about this one.
   public void testSurfaceNormalForRamp90()
   {
      testSurfaceNormalsForAnyRamp(ramp90, expectedSurfaceNormalRamp90, pointsOnRamp90, expectedSurfaceNormalOnOtherFacesRamp90, pointsOnOtherFacesRamp90);
   }

   private void testHeightAtRampForAnyRamp(Point3d[] pointsOnRamp, RotatableRampTerrainObject ramp)
   {
      for (int i = 0; i < pointsOnRamp.length; i++)
      {
         String message = "Expected Height For point " + pointsOnRamp[i].getX() + " " + pointsOnRamp[i].getY() + " " + pointsOnRamp[i].getZ();
         assertEquals(message, pointsOnRamp[i].getZ(), ramp.heightAt(pointsOnRamp[i].getX(), pointsOnRamp[i].getY(), pointsOnRamp[i].getZ()), epsilon);
      }
   }

   private void testSurfaceNormalsForAnyRamp(RotatableRampTerrainObject ramp, Vector3d expectedRampSurfaceNormal, Point3d[] pointsOnRamp,
           Vector3d[] expectedSurfaceNormalOnOtherFaces, Point3d[] pointsOnOtherRampFaces)
   {
      expectedRampSurfaceNormal.normalize();

      for (int i = 0; i < pointsOnRamp.length; i++)
      {
         Vector3d normal = new Vector3d();
         ramp.surfaceNormalAt(pointsOnRamp[i].getX(), pointsOnRamp[i].getY(), pointsOnRamp[i].getZ(), normal);
         String message = "Normal for point " + pointsOnRamp[i].getX() + " " + pointsOnRamp[i].getY() + " " + pointsOnRamp[i].getZ();
         JUnitTools.assertTuple3dEquals(message, expectedRampSurfaceNormal, normal, epsilon);
      }

      for (int i = 0; i < pointsOnOtherRampFaces.length; i++)
      {
         expectedSurfaceNormalOnOtherFaces[i].normalize();
         Vector3d normal = new Vector3d();
         ramp.surfaceNormalAt(pointsOnOtherRampFaces[i].getX(), pointsOnOtherRampFaces[i].getY(), pointsOnOtherRampFaces[i].getZ(), normal);
         String message = "Normal for point " + pointsOnOtherRampFaces[i].getX() + " " + pointsOnOtherRampFaces[i].getY() + " "
                          + pointsOnOtherRampFaces[i].getZ();
         JUnitTools.assertTuple3dEquals(message, expectedSurfaceNormalOnOtherFaces[i], normal, epsilon);
      }
   }


}
