package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.tools.testing.JUnitTools;

public class ShipCorridorGroundProfileTest 
{
   private ShipCorridorGroundProfile groundProfile;
   private final double epsilon = 1e-6;
   private final boolean debug = false;

   @Before
   public void setUp()
   {
      groundProfile = new ShipCorridorGroundProfile(100.0, -10.0, 5.0, -5.0, 0.8, -0.8, 0.0, 3.0, Math.toRadians( 2.0 ));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSurfaceNormalAlongYAxis()
   {
      int nSteps = 1000;
      BoundingBox3d boundingBox = groundProfile.getBoundingBox();
      
      double yStep = (boundingBox.getYMax() - boundingBox.getYMin()) / nSteps;
      double dy = 1e-8;
      double x = (boundingBox.getXMax() - boundingBox.getXMin()) / 2.0;
      double z = 0.0;
      double y = -2.5; //Start on the left side and move to the right side

      for (int i = 0; i < nSteps; i++)
      {
         y = y + yStep;
         double initialHeight;
         double finalHeight;

         initialHeight = groundProfile.heightAt(x, y, z);
         finalHeight = groundProfile.heightAt(x, y + yStep, z);

         if (((y + yStep) < 5.0))
         {
            //Normal
            double dzdy = (groundProfile.heightAt(x, y + yStep, z) - groundProfile.heightAt(x, y, z)) / yStep;

            //Numerical Surface normal
            Vector3d numericalSurfaceNormal = new Vector3d(0.0, -dzdy, 1.0);
            numericalSurfaceNormal.normalize();

            //Other Surface normal
            Vector3d surfaceNormalFromGroundProfile = new Vector3d();
            Point3d intersection = new Point3d();
            groundProfile.closestIntersectionAndNormalAt(x, y, z, intersection, surfaceNormalFromGroundProfile);
            
            if(((initialHeight == 3.0)&&(finalHeight != 3.0)) || ((initialHeight != 3.0)&&(finalHeight == 3.0)) || ((initialHeight != 0.0)&&(finalHeight == 0.0)) || ((initialHeight == 0.0)&&(finalHeight != 0.0)))
            {
               surfaceNormalFromGroundProfile.setX(0.0);
               surfaceNormalFromGroundProfile.setY(0.0);
               surfaceNormalFromGroundProfile.setZ(1.0);
               numericalSurfaceNormal.setX(0.0);
               numericalSurfaceNormal.setY(0.0);
               numericalSurfaceNormal.setZ(1.0);         
            }
            
            if(debug)
            {
               System.out.println("y :" + y + "   y + dy :" + (y + yStep));
               System.out.println("Height initial :" + groundProfile.heightAt(x, y, z));
               System.out.println("Height final   :" + groundProfile.heightAt(x, y + yStep, z));
               System.out.println("dzdy   :" + -dzdy);
               System.out.println("Normal Surface  : " + surfaceNormalFromGroundProfile);
               System.out.println("Normal Numerical: " + numericalSurfaceNormal);
               System.out.println("\n\n");
            }
  
            JUnitTools.assertTuple3dEquals(numericalSurfaceNormal, surfaceNormalFromGroundProfile, epsilon);
         }
      }
   }
}
