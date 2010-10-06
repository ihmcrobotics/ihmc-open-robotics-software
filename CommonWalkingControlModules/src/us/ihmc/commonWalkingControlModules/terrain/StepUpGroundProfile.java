package us.ihmc.commonWalkingControlModules.terrain;


import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.GroundProfile;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class StepUpGroundProfile implements GroundProfile
{
   private final double xMin, xMax, yMin, yMax;
   private final double xTiles = 1.0, yTiles = 1.0;
   
   private final DoubleYoVariable groundXStep, groundZStep;
   
   public StepUpGroundProfile(YoVariableRegistry registry)
   {
      this.xMin = -1.0;
      this.xMax = 4.0;
      this.yMin = -1.0;
      this.yMax = 1.0;
      
      groundXStep = new DoubleYoVariable("groundXStep", registry);
      groundZStep = new DoubleYoVariable("groundZStep", registry);
      
      groundXStep.set(1.4);
      groundZStep.set(0.10); //0.07);
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersection, Vector3d normal)
   {
      closestIntersectionTo(x, y, z, intersection);
      surfaceNormalAt(x, y, z, normal);

   }

   public void closestIntersectionTo(double x, double y, double z, Point3d intersection)
   {
      intersection.set(x, y, heightAt(x, y, z));
   }

   public double getXMax()
   {
      return xMax;
   }

   public double getXMin()
   {
      return xMin;
   }

   public double getXTiles()
   {
      return xTiles;
   }

   public double getYMax()
   {
      return yMax;
   }

   public double getYMin()
   {
      return yMin;
   }

   public double getYTiles()
   {
      return yTiles;
   }

   public double heightAt(double x, double y, double z)
   {
      if (x > groundXStep.getDoubleValue()) return groundZStep.getDoubleValue();
      return 0.0;
   }

   public boolean isClose(double x, double y, double z)
   {
      return true;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      normal.set(0.0, 0.0, 1.0);
   }

}

