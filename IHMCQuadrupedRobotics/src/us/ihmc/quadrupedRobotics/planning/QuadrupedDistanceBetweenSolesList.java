package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Point3d;

public class QuadrupedDistanceBetweenSolesList extends QuadrantDependentList<QuadrantDependentList<MutableDouble>>
{
   private QuadrantDependentList<MutableDouble> quadrantDependentList = new QuadrantDependentList<>();
   private Point3d solePointA;
   private Point3d solePointB;

   public QuadrupedDistanceBetweenSolesList()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         quadrantDependentList.set(quadrant, new MutableDouble(0.0));
      }
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         this.set(quadrant, quadrantDependentList);
      }
      solePointA = new Point3d();
      solePointB = new Point3d();
   }

   public double get(RobotQuadrant key1, RobotQuadrant key2)
   {
      return this.get(key1).get(key2).doubleValue();
   }

   public void compute(QuadrantDependentList<FramePoint> solePositions, QuadrupedReferenceFrames referenceFrames)
   {
      for (RobotQuadrant quadrantA : RobotQuadrant.values())
      {
         QuadrantDependentList<MutableDouble> quadrantDependentList = new QuadrantDependentList<>();
         for (RobotQuadrant quadrantB : RobotQuadrant.values())
         {
            solePositions.get(quadrantA).changeFrame(referenceFrames.getBodyFrame());
            solePositions.get(quadrantB).changeFrame(referenceFrames.getBodyFrame());
            Point3d solePointA = solePositions.get(quadrantA).getPoint();
            Point3d solePointB = solePositions.get(quadrantB).getPoint();
            quadrantDependentList.set(quadrantB, new MutableDouble(solePointA.distance(solePointB)));
         }
         this.set(quadrantA, quadrantDependentList);
      }
   }

   public void compute(QuadrantDependentList<Point3d> solePositions)
   {
      for (RobotQuadrant quadrantA : RobotQuadrant.values())
      {
         QuadrantDependentList<MutableDouble> quadrantDependentList = new QuadrantDependentList<>();
         for (RobotQuadrant quadrantB : RobotQuadrant.values())
         {
            solePointA.set(solePositions.get(quadrantA));
            solePointB.set(solePositions.get(quadrantB));
            quadrantDependentList.set(quadrantB, new MutableDouble(solePointA.distance(solePointB)));
         }
         this.set(quadrantA, quadrantDependentList);
      }
   }

   public boolean epsilonEquals(QuadrupedDistanceBetweenSolesList other, double epsilon)
   {
      boolean output = true;
      for (RobotQuadrant quadrantA : RobotQuadrant.values())
      {
         for (RobotQuadrant quadrantB : RobotQuadrant.values())
         {
            if (output)
            {
               output = output && MathTools.epsilonEquals(this.get(quadrantA, quadrantB), other.get(quadrantA, quadrantB), epsilon);
            }
            else
            {
               return output;
            }
         }
      }
      return output;
   }
}
