package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;
import us.ihmc.robotics.MathTools;

public class ConvexPolytopeConstructorTest
{
   @Before
   public void setup()
   {

   }
   
   @After
   public void tearDown()
   {
      
   }
   
   @Test(timeout = 10000)
   public void testCylinderConstructor()
   {
      int numberOfSide = 20;
      ExtendedConvexPolytope cylinder = ConvexPolytopeConstructor.constructCylinder(new Point3D(), 10.0, 2.0, numberOfSide);
      assertTrue(cylinder != null);
      assertTrue(cylinder.getNumberOfFaces() == numberOfSide + 2);
      for (int j = 0; j < cylinder.getNumberOfFaces(); j++)
      {
         ConvexPolytopeFace face = cylinder.getFace(j);
         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            assertTrue("Null twin edge for edge: " + face.getEdge(i).toString() + " on face: " + face.toString(), face.getEdge(i).getTwinHalfEdge() != null);
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getOriginVertex() == face.getEdge(i).getDestinationVertex());
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getDestinationVertex() == face.getEdge(i).getOriginVertex());
         }
      }
   }
   
   @Test
   public void testCylindericalPointsListCreation()
   {
      double xCenter = 1.0;
      double yCenter = 5.0;
      double radius = 2.0;
      double zCenter = radius;
      double length = 4.0;
      int curveDiscretization = 20;
      ArrayList<Point3D> pointList = ConvexPolytopeConstructor.getCollisionMeshPointsForCylinder(xCenter, yCenter, zCenter, Axis.Z, radius, length, curveDiscretization);
      testCylindericalPoints(xCenter, yCenter, zCenter, Axis.Z, radius, length, curveDiscretization, pointList);
      pointList = ConvexPolytopeConstructor.getCollisionMeshPointsForCylinder(xCenter, yCenter, zCenter, Axis.X, radius, length, curveDiscretization);
      testCylindericalPoints(xCenter, yCenter, zCenter, Axis.X, radius, length, curveDiscretization, pointList);
      pointList = ConvexPolytopeConstructor.getCollisionMeshPointsForCylinder(xCenter, yCenter, zCenter, Axis.Y, radius, length, curveDiscretization);
      testCylindericalPoints(xCenter, yCenter, zCenter, Axis.Y, radius, length, curveDiscretization, pointList);
   }
   
   private void testCylindericalPoints(double xCenter, double yCenter, double zCenter, Axis axis, double radius, double length, int curveDiscretization, ArrayList<? extends Point3DReadOnly> pointList)
   {
      assertEquals(pointList.size(), curveDiscretization * 2);
      Vector3D axisVector = new Vector3D(axis.getAxisVector());
      axisVector.sub(1.0, 1.0, 1.0);
      axisVector.negate();
      Vector3D tempVectorForTesting = new Vector3D();
      Point3D center = new Point3D(xCenter, yCenter, zCenter);
      for(int i = 0; i < pointList.size(); i++)
      {
         tempVectorForTesting.sub(pointList.get(i), center);
         assertTrue("", MathTools.epsilonEquals(Math.abs(axis.getAxisVector().dot(tempVectorForTesting)), length / 2.0, Epsilons.ONE_BILLIONTH));
         tempVectorForTesting.scale(axisVector.getX(), axisVector.getY(), axisVector.getZ());
         double radiusToTest = Math.sqrt(tempVectorForTesting.dot(tempVectorForTesting));
         assertTrue("Wanted: " + radius + " Got: " + radiusToTest, MathTools.epsilonEquals(radius, radiusToTest, Epsilons.ONE_BILLIONTH));
      }
   }
}
