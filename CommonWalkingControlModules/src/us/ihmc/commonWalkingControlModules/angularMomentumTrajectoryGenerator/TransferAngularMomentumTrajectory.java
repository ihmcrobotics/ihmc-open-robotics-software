package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class TransferAngularMomentumTrajectory implements AngularMomentumTrajectory
{
   private List<AngularMomentumTrajectoryPoint> waypoints;
   private List<YoFrameTrajectory3D> polynomialCoefficients;
   private YoInteger numberOfSegments;
   private YoInteger currentSegment;
   private int maximumNumberOfCoefficients = 4;
   private YoFrameVector momentum;
   private YoFrameVector torque;
   private YoFrameVector rotatum;
   
   public TransferAngularMomentumTrajectory(String namePrefix, YoVariableRegistry registry, ReferenceFrame referenceFrame, int maxNumberOfWaypoints)
   {
      polynomialCoefficients = new ArrayList<>(maxNumberOfWaypoints);
      waypoints = new ArrayList<>(maxNumberOfWaypoints);
      momentum = new YoFrameVector(namePrefix + "Positon", referenceFrame, registry);
      torque = new YoFrameVector(namePrefix + "Velocity", referenceFrame, registry);
      rotatum = new YoFrameVector(namePrefix + "Acceleration", referenceFrame, registry);
      numberOfSegments = new YoInteger(namePrefix + "NumberOfSegments", registry);
      currentSegment = new YoInteger(namePrefix + "CuurentSegment", registry);
      
      for(int i = 0; i < maxNumberOfWaypoints; i++)
      {
         AngularMomentumTrajectoryPoint waypoint = new AngularMomentumTrajectoryPoint(namePrefix + "Waypoint"+i, registry, referenceFrame);
         waypoints.add(waypoint);
         YoFrameTrajectory3D trajectory = new YoFrameTrajectory3D(namePrefix, maximumNumberOfCoefficients, referenceFrame, registry);
         polynomialCoefficients.add(trajectory);
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
      for (int i = 0; i < polynomialCoefficients.size(); i++)
         polynomialCoefficients.get(i).reset();
   }

   @Override
   public void update(double timeInState)
   {
      for(int i = 0; i < numberOfSegments.getIntegerValue(); i++)
      {
         if(timeInState > polynomialCoefficients.get(i).getInitialTime() && timeInState < polynomialCoefficients.get(i).getFinalTime())
         {
            currentSegment.set(i);
            break;
         }
      }
      polynomialCoefficients.get(currentSegment.getIntegerValue()).compute(timeInState);
   }

   @Override
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack)
   {
      update(timeInState);
      desiredAngularMomentumToPack.setIncludingFrame(polynomialCoefficients.get(currentSegment.getIntegerValue()).getFramePosition());
   }

   @Override
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack)
   {
      update(timeInState, desiredAngularMomentumToPack);
      desiredTorqueToPack.setIncludingFrame(polynomialCoefficients.get(currentSegment.getIntegerValue()).getFrameVelocity());
   }

   @Override
   public List<YoFrameTrajectory3D> getPolynomials()
   {
      return polynomialCoefficients;
   }

   public void set(List<AngularMomentumTrajectoryPoint> samplePoints)
   {
      // TODO Auto-generated method stub

   }

   public void computeFromCoPWaypoints(double t0, double tFinal, FramePoint zInitial, FramePoint zRefPoint1, FramePoint zRefPoint2, FramePoint zFinal)
   {
      polynomialCoefficients.get(0).setCubicBezier(t0, tFinal, zInitial, zRefPoint1, zRefPoint2, zFinal);
      numberOfSegments.set(1);
   }
   
   @Override
   public int getNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }

}
