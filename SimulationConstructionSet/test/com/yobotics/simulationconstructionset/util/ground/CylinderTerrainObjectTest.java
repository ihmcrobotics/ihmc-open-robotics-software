package com.yobotics.simulationconstructionset.util.ground;

import static org.junit.Assert.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

public class CylinderTerrainObjectTest
{
   CylinderTerrainObject verticalCylinder;
   CylinderTerrainObject horizontalCylinder;
   AppearanceDefinition app=YoAppearance.Red();
   double height;
   double radius;
   double errEpsilon = 1e-14;
   double testDelta = .0001;
   
   CylinderTerrainObject slopedRotatedCylinder;
   double angleAxisDownRadians;
   double angleAxisFromXRadians;
   
   Vector3d translatedCenter=new Vector3d(5, 3, 1.5);    
   CylinderTerrainObject translatedVerticalCylinder;
   CylinderTerrainObject translatedHorizontalCylinder;
   
   double tallHeight;
   CylinderTerrainObject rot90TallHorizontalCylinder;
   CylinderTerrainObject translatedRot90TallHorizontalCylinder;
   
   @Before
   public void setup()
   {
      Vector3d center=new Vector3d(0, 0, 0);      
      double slopeDegrees=0.0;
      double yawDegrees=0.0;
      height=2.0;
      radius=1.0;
      verticalCylinder = new CylinderTerrainObject(center,slopeDegrees,yawDegrees, height, radius, app);
      translatedVerticalCylinder = new CylinderTerrainObject(translatedCenter,slopeDegrees,yawDegrees, height, radius, app);
      slopeDegrees=90.0;
      horizontalCylinder = new CylinderTerrainObject(center,slopeDegrees,yawDegrees, height, radius, app);
      translatedHorizontalCylinder = new CylinderTerrainObject(translatedCenter,slopeDegrees,yawDegrees, height, radius, app);
      slopeDegrees = 45.0;
      yawDegrees = 30.0;
      angleAxisDownRadians=Math.toRadians(slopeDegrees);
      angleAxisFromXRadians = Math.toRadians(yawDegrees);
      slopedRotatedCylinder =  new CylinderTerrainObject(center,slopeDegrees,yawDegrees, height, radius, app);
      
      slopeDegrees = 90.0;
      yawDegrees = 90.0;
      tallHeight = 2*height;
      rot90TallHorizontalCylinder = new CylinderTerrainObject(center,slopeDegrees,yawDegrees, tallHeight, radius, app);
      translatedRot90TallHorizontalCylinder = new CylinderTerrainObject(translatedCenter,slopeDegrees,yawDegrees, tallHeight, radius, app);
   }
   
   @Test
   public void testHeightAtTranslatedRot90TallHorizontalCylinderJustInsideAndOutside()
   {
      
      double expectedHeight = radius+translatedCenter.z;
      double expectedMiss = 0.0;
      double[] signY = {0,  -1, 1};
      boolean[] isEdge = {false, true, true};
      
      for(int i=0;i<signY.length;i++)
      {
            double testX=0.0 + +translatedCenter.x;
            double testY=signY[i]*(tallHeight/2-testDelta) + +translatedCenter.y;
            double testZ=expectedHeight+1.0;
            
            assertEquals(expectedHeight,translatedRot90TallHorizontalCylinder.heightAt(testX, testY, testZ),errEpsilon);

            if(isEdge[i])
            {
               testY=signY[i]*(tallHeight/2+testDelta)+translatedCenter.y;
               assertEquals(expectedMiss,translatedRot90TallHorizontalCylinder.heightAt(testX, testY, testZ),errEpsilon);
            }
      }
   }
      
   @Test
   public void testHeightAtRot90TallHorizontalCylinderJustInsideAndOutside()
   {
      
      double expectedHeight = radius;
      double expectedMiss = 0.0;
      double[] signY = {0,  -1, 1};
      boolean[] isEdge = {false, true, true};
      
      for(int i=0;i<signY.length;i++)
      {
            double testX=0.0;
            double testY=signY[i]*(tallHeight/2-testDelta);
            double testZ=expectedHeight+1.0;
            
            assertEquals(expectedHeight,rot90TallHorizontalCylinder.heightAt(testX, testY, testZ),errEpsilon);

            if(isEdge[i])
            {
               testY=signY[i]*(tallHeight/2+testDelta);
               assertEquals(expectedMiss,rot90TallHorizontalCylinder.heightAt(testX, testY, testZ),errEpsilon);
            }
      }
   }
      
   @Test
   public void testHeightAtTranslatedVerticalCylinderJustInside()
   {
      
      double expectedHeight = translatedCenter.z+height/2;
      double[] signX = {-1,  0, 1, 0};
      double[] signY = {0,  -1, 0, 1};
      
      for(int i=0;i<signX.length;i++)
      {
            double testX=signX[i]*(radius-testDelta)+translatedCenter.x;
            double testY=signY[i]*(radius-testDelta)+translatedCenter.y;
            double testZ=expectedHeight+1.0;
            
            assertEquals(expectedHeight,translatedVerticalCylinder.heightAt(testX, testY, testZ),errEpsilon);
      }
   }
   
   @Test
   public void testHeightAtTranslatedHorizontalCylinderJustInside()
   {
      
      double expectedHeight = translatedCenter.z+radius;
      double[] signX = {-1,  0, 1};
      
      for(int i=0;i<signX.length;i++)
      {
            double testX=signX[i]*(height/2-testDelta)+translatedCenter.x;
            double testY=translatedCenter.y;
            double testZ=expectedHeight+1.0;
            
            assertEquals(expectedHeight,translatedHorizontalCylinder.heightAt(testX, testY, testZ),errEpsilon);
      }
   }
   
   @Test
   public void testHeightAtTranslatedVerticalCylinderJustOutside()
   {
      
      double expectedHeight = 0;
      double[] signX = {-1,  0, 1, 0};
      double[] signY = {0,  -1, 0, 1};
      
      for(int i=0;i<signX.length;i++)
      {
            double testX=signX[i]*(radius+testDelta)+translatedCenter.x;
            double testY=signY[i]*(radius+testDelta)+translatedCenter.y;
            double testZ=expectedHeight+1.0;
            
            assertEquals(expectedHeight,translatedVerticalCylinder.heightAt(testX, testY, testZ),errEpsilon);
      }
   }
   
   @Test
   public void testHeightAtTranslatedHorizontalCylinderJustOutside()
   {
      
      double expectedHeight = 0;
      double[] signX = {-1, 1};
      
      for(int i=0;i<signX.length;i++)
      {
            double testX=signX[i]*(height/2+testDelta)+translatedCenter.x;
            double testY=translatedCenter.y;
            double testZ=expectedHeight+1.0;
            
            assertEquals(expectedHeight,translatedHorizontalCylinder.heightAt(testX, testY, testZ),errEpsilon);
      }
   }
   
   @Test
   public void testHeightAtVerticalCylinderOutside()
   {
      assertEquals(0,verticalCylinder.heightAt(height/2*1.5, 0, 0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderOutside2()
   {
      assertEquals(0,verticalCylinder.heightAt(-height/2*1.5, 0, 0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderOutside3()
   {
      assertEquals(0,verticalCylinder.heightAt(0,-height/2*1.5, 0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderOutside4()
   {
      assertEquals(0,verticalCylinder.heightAt(0,height/2*1.5, 0),errEpsilon);
   }

   @Test
   public void testHeightAtHorizontalCylinderOutsideToTopEnd()
   {
      assertEquals(0,horizontalCylinder.heightAt(height/2*1.5, 0, 0),errEpsilon);//fails with only test outside to side
   }

   @Test
   public void testHeightAtHorizontalCylinderOutsideToBottomEnd()
   {
      assertEquals(0,horizontalCylinder.heightAt(-height/2*1.5, 0, 0),errEpsilon);//fails with only test outside to side
   }

   @Test
   public void testHeightAtHorizontalCylinderOutsideToSide()
   {
      assertEquals(0,horizontalCylinder.heightAt(0, radius*1.5, 0),errEpsilon);
   }

   @Test
   public void testHeightAtHorizontalCylinderOutsideToOtherSide()
   {
      assertEquals(0,horizontalCylinder.heightAt(0, -radius*1.5, 0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderTopCenter()
   {
      assertEquals(height/2.0,verticalCylinder.heightAt(0, 0, height/2.0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderBottomCenter()
   {
      assertEquals(-height/2.0,verticalCylinder.heightAt(0, 0, -height/2.0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderTop0()
   {
      assertEquals(height/2.0,verticalCylinder.heightAt(radius/2, 0, height/2.0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderTop1()
   {
      assertEquals(height/2.0,verticalCylinder.heightAt(-radius/2, 0, height/2.0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderTop2()
   {
      assertEquals(height/2.0,verticalCylinder.heightAt(0, radius/2, height/2.0),errEpsilon);
   }

   @Test
   public void testHeightAtVerticalCylinderTop3()
   {
      assertEquals(height/2.0,verticalCylinder.heightAt(0, -radius/2, height/2.0),errEpsilon);
   }

   @Test
   public void testHeightAtHorizontalCylinderTopCenter()
   {
      assertEquals(radius,verticalCylinder.heightAt(0, 0, radius),errEpsilon);
   }

   @Test
   public void testHeightAtHorizontalCylinderBottomCenter()
   {
      assertEquals(-radius,verticalCylinder.heightAt(0, 0, -radius),errEpsilon);
   }

   @Test
   public void testHeightAtHorizontalCylinderTopForward()
   {
      assertEquals(radius,verticalCylinder.heightAt(height/4, 0, radius),errEpsilon);
   }

   @Test
   public void testHeightAtHorizontalCylinderTopBackward()
   {
      assertEquals(radius,verticalCylinder.heightAt(-height/4, 0, radius),errEpsilon);
   }

   @Test
   public void testHeightAtHorizontalCylinderTopSide()
   {
      double expectedHeightOnCircle = Math.sqrt(radius*radius - (radius/2)*(radius/2));
      assertEquals(expectedHeightOnCircle,horizontalCylinder.heightAt(height/4, radius/2, radius),errEpsilon);
   }

   @Test
   public void testHeightAtSlopedRotatedTwoSidesTop()
   {
      double distanceAlongAxis = height/2;
      double dCenterToContact=Math.sqrt(distanceAlongAxis*distanceAlongAxis + radius*radius);
      double angleFromAxisToD=Math.atan(radius/distanceAlongAxis);
      double dHorizontal = dCenterToContact*Math.cos(angleAxisDownRadians+angleFromAxisToD);
      double dVertical  = dCenterToContact*Math.sin(angleAxisDownRadians+angleFromAxisToD);
      double x=dHorizontal*Math.cos(angleAxisFromXRadians);
      double y=dHorizontal*Math.sin(angleAxisFromXRadians);
      double z=dVertical;
      
      assertEquals(z,slopedRotatedCylinder.heightAt(x, y, z+1),errEpsilon);
   }

   @Test
   public void testHeightAtSlopedRotatedTwoSidesBottom()
   {
      double distanceAlongAxis = height/2;
      double dCenterToContact=Math.sqrt(distanceAlongAxis*distanceAlongAxis + radius*radius);
      double angleFromAxisToD=Math.atan(radius/distanceAlongAxis);//Should Be 45 for expectedHeight below to work
      double dHorizontal = dCenterToContact*Math.cos(angleAxisDownRadians+angleFromAxisToD);
      double dVertical  = dCenterToContact*Math.sin(angleAxisDownRadians+angleFromAxisToD);
      double x=dHorizontal*Math.cos(angleAxisFromXRadians);
      double y=dHorizontal*Math.sin(angleAxisFromXRadians);
      double z=dVertical;
      
      double expectedHeight = z-Math.sqrt(2*(2*radius)*(2*radius));
      assertEquals(expectedHeight,slopedRotatedCylinder.heightAt(x, y, -1),errEpsilon);
   }

   @Test
   public void testHeightAtSlopedRotatedEndAndSideTop()
   {
      double distanceAlongAxis = height/2;
      double dHorizontal = distanceAlongAxis*Math.cos(angleAxisDownRadians);
      double dVertical  = distanceAlongAxis*Math.sin(angleAxisDownRadians);
      double x=dHorizontal*Math.cos(angleAxisFromXRadians);
      double y=dHorizontal*Math.sin(angleAxisFromXRadians);
      double z=dVertical;
      
      double expectedHeight = z;
      assertEquals(expectedHeight,slopedRotatedCylinder.heightAt(x, y, 2),errEpsilon);
   }

   @Test
   public void testHeightAtSlopedRotatedEndAndSideBottom()
   {
      double distanceAlongAxis = height/2;
      double dHorizontal = distanceAlongAxis*Math.cos(angleAxisDownRadians);
      double dVertical  = distanceAlongAxis*Math.sin(angleAxisDownRadians);
      double x=dHorizontal*Math.cos(angleAxisFromXRadians);
      double y=dHorizontal*Math.sin(angleAxisFromXRadians);
      double z=dVertical;
      
      double expectedHeight = z-Math.sqrt(radius*radius*2);
      assertEquals(expectedHeight,slopedRotatedCylinder.heightAt(x, y, -2),errEpsilon);
   }

}
