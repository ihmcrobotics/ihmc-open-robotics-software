package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;

import java.util.ArrayList;
import java.util.List;

/**
 * This is used to define contact transitions for quadrupeds. This essentially converts {@link TimeInterval} to step transition events, including
 * touchdowns and lift offs. This is used then to convert to contact sequences.
 */
class QuadrupedStepTransition
{
   public static final double sameTimeEpsilon = 1e-3;

   private final List<RobotQuadrant> transitionQuadrants = new ArrayList<>();
   private final List<QuadrupedStepTransitionType> transitionTypes = new ArrayList<>();
   private final QuadrantDependentList<Point3D> transitionPositions = new QuadrantDependentList<>();
   private double transitionTime = Double.MAX_VALUE;

   public QuadrupedStepTransition()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         Point3D transitionPosition = new Point3D();
         transitionPosition.setToNaN();
         transitionPositions.put(robotQuadrant, transitionPosition);
      }
   }

   public void reset()
   {
      transitionTime = Double.MAX_VALUE;
      transitionQuadrants.clear();
      transitionTypes.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         transitionPositions.get(robotQuadrant).setToNaN();
      }
   }

   public void setTransitionTime(double time)
   {
      this.transitionTime = time;
   }

   public void addTransition(QuadrupedStepTransitionType transitionType, RobotQuadrant transitionQuadrant, Point3DReadOnly transitionPosition)
   {
      transitionTypes.add(transitionType);
      transitionQuadrants.add(transitionQuadrant);
      transitionPositions.get(transitionQuadrant).set(transitionPosition);
   }

   public void addTransition(QuadrupedStepTransition other)
   {
      if (!MathTools.epsilonEquals(transitionTime, other.transitionTime, sameTimeEpsilon))
         throw new IllegalArgumentException("These transitions occur at different times!");

      for (int i = 0; i < other.transitionQuadrants.size(); i++)
      {
         RobotQuadrant addingQuadrant = other.transitionQuadrants.get(i);
         transitionQuadrants.add(addingQuadrant);
         transitionTypes.add(other.transitionTypes.get(i));
         transitionPositions.get(addingQuadrant).set(other.transitionPositions.get(addingQuadrant));
      }
   }

   public double getTransitionTime()
   {
      return transitionTime;
   }

   public int getNumberOfFeetInTransition()
   {
      return transitionQuadrants.size();
   }

   public QuadrupedStepTransitionType getTransitionType(int transitionNumber)
   {
      return transitionTypes.get(transitionNumber);
   }

   public RobotQuadrant getTransitionQuadrant(int transitionNumber)
   {
      return transitionQuadrants.get(transitionNumber);
   }

   public Point3DReadOnly getTransitionPosition(RobotQuadrant transitionQuadrant)
   {
      return transitionPositions.get(transitionQuadrant);
   }

   List<RobotQuadrant> getTransitionQuadrants()
   {
      return transitionQuadrants;
   }
}
