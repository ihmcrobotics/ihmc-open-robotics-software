package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class AngularMomentumTrajectory implements AngularMomentumTrajectoryInterface
{
   private List<AngularMomentumTrajectoryPoint> waypoints;
   private List<YoFrameTrajectory3D> trajectories;
   private YoInteger numberOfSegments;
   private YoInteger currentSegment;
   private int maximumNumberOfCoefficients = 4;
   private YoFrameVector momentum;
   private YoFrameVector torque;
   private YoFrameVector rotatum;

   public AngularMomentumTrajectory(String namePrefix, int stepNumber, WalkingTrajectoryType type, YoVariableRegistry registry, ReferenceFrame referenceFrame,
                                    int maxNumberOfWaypoints)
   {
      trajectories = new ArrayList<>(maxNumberOfWaypoints - 1);
      waypoints = new ArrayList<>(maxNumberOfWaypoints);
      momentum = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Positon", referenceFrame, registry);
      torque = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Velocity", referenceFrame, registry);
      rotatum = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Acceleration", referenceFrame, registry);
      numberOfSegments = new YoInteger(namePrefix + stepNumber + type.toString() + "NumberOfSegments", registry);
      currentSegment = new YoInteger(namePrefix + stepNumber + type.toString() + "CuurentSegment", registry);

      for (int i = 0; i < maxNumberOfWaypoints; i++)
      {
         AngularMomentumTrajectoryPoint waypoint = new AngularMomentumTrajectoryPoint(namePrefix + "Waypoint" + i, registry, referenceFrame);
         waypoints.add(waypoint);
      }
      for (int i = 0; i < maxNumberOfWaypoints - 1; i++)
      {
         YoFrameTrajectory3D trajectory = new YoFrameTrajectory3D(namePrefix, maximumNumberOfCoefficients, referenceFrame, registry);
         trajectories.add(trajectory);
      }
   }

   @Override
   public void reset()
   {
      momentum.setToZero();
      torque.setToZero();
      rotatum.setToZero();
      for (int i = 0; i < waypoints.size(); i++)
         waypoints.get(i).reset();
      for (int i = 0; i < trajectories.size(); i++)
         trajectories.get(i).reset();
   }

   @Override
   public void update(double timeInState)
   {
      getCurrentSegmentFromTimeInState(timeInState);
      trajectories.get(currentSegment.getIntegerValue()).compute(timeInState);
   }

   private void getCurrentSegmentFromTimeInState(double timeInState)
   {
      int index = 0;
      for (; index < getNumberOfSegments(); index++)
         if (trajectories.get(index).timeIntervalContains(timeInState))
            break;
      if(index == getNumberOfSegments())
         currentSegment.set(-1);
      else
         currentSegment.set(index);
   }

   @Override
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack)
   {
      update(timeInState);
      desiredAngularMomentumToPack.setIncludingFrame(trajectories.get(currentSegment.getIntegerValue()).getFramePosition());
   }

   @Override
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack)
   {
      update(timeInState, desiredAngularMomentumToPack);
      desiredTorqueToPack.setIncludingFrame(trajectories.get(currentSegment.getIntegerValue()).getFrameVelocity());
   }

   @Override
   public List<YoFrameTrajectory3D> getPolynomials()
   {
      return trajectories;
   }

   public void set(List<AngularMomentumTrajectoryPoint> samplePoints)
   {
      // TODO Auto-generated method stub

   }

   public void computeFromCoPWaypoints(double t0, double tFinal, FramePoint zInitial, FramePoint zRefPoint1, FramePoint zRefPoint2, FramePoint zFinal)
   {
      trajectories.get(0).setCubicBezier(t0, tFinal, zInitial, zRefPoint1, zRefPoint2, zFinal);
      numberOfSegments.set(1);
   }

   @Override
   public int getNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }

}
