package us.ihmc.manipulation.planning.forwaypoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector3D;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SO3TrajectoryPointCalculatorTwo
{
   public double firstTrajectoryPointTime;
   
   public List<Quaternion> trajectoryPointsOrientation = new ArrayList<>();
   public List<Double> trajectoryPointsTime = new ArrayList<>();
   
   
   public List<Vector3D> trajectoryPointsAngularVelocity = new ArrayList<>();
   
   public SO3TrajectoryPointCalculatorTwo()
   {
      
   }
   
   public void clear()
   {
      trajectoryPointsOrientation.clear();
      trajectoryPointsTime.clear();
   }
   
   public void setFirstTrajectoryPointTime(double firstTrajectoryPointTime)
   {
      this.firstTrajectoryPointTime = firstTrajectoryPointTime;
   }
   
   public void appendTrajectoryPointOrientation(double time, Quaternion quaternion)
   {
      trajectoryPointsOrientation.add(quaternion);
      trajectoryPointsTime.add(time);
   }
   
   public void compute()
   {
      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      
      int numberOfTrajectoryPoints = trajectoryPointsOrientation.size();
      for(int i=0;i<numberOfTrajectoryPoints;i++)
      {
         Vector3D orientation = new Vector3D();
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(trajectoryPointsOrientation.get(i), orientation);
         
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(new Point3D(orientation));
      }
      
      double[] trajectoryTimes = new double[numberOfTrajectoryPoints];
      
      for(int i=0;i<numberOfTrajectoryPoints;i++)
      {
         trajectoryTimes[i] = trajectoryPointsTime.get(i);
      }
      
      euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTimes);
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(false);

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
         
      trajectoryPointsAngularVelocity.clear();
      for(int i=0;i<numberOfTrajectoryPoints;i++)
      {
         Point3D orientationYawPitchRoll = new Point3D();
         Vector3D angularVelocityYawPitchRoll = new Vector3D();
         
         double time = trajectoryPoints.get(i).get(orientationYawPitchRoll, angularVelocityYawPitchRoll);
         
         PrintTools.info(""+i+" "+ orientationYawPitchRoll +" " + angularVelocityYawPitchRoll);
         
         trajectoryPointsAngularVelocity.add(angularVelocityYawPitchRoll);
      }
   }
   
   public List<Vector3D> getTrajectoryPointsAngularVelocity()
   {
      return trajectoryPointsAngularVelocity;
   }
   
   public Vector3D getAngularVelocity(double time)
   {      
      Vector3D angularVelocity = new Vector3D();

      /* 
       * For debug in SO3TrajectoryPointCalculatorVisualizer
       */
      
      return angularVelocity;
   }
   
   public Quaternion getOrientation(double time)
   {  
      int arrayIndex = 0;
      
      int numberOfTrajectoryPoints = trajectoryPointsTime.size();
      for(int i=0;i<numberOfTrajectoryPoints-1;i++)
      {
         if(trajectoryPointsTime.get(i) <= time && time <= trajectoryPointsTime.get(i+1))
            arrayIndex = i;
      }
      
      double localStartTime = trajectoryPointsTime.get(arrayIndex);
      double localEndTime = trajectoryPointsTime.get(arrayIndex+1);
      
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      
      FrameOrientation initialOrientation = new FrameOrientation(worldFrame, trajectoryPointsOrientation.get(arrayIndex));
      FrameVector3D initialAngularVelocity = new FrameVector3D(worldFrame, trajectoryPointsAngularVelocity.get(arrayIndex));
      FrameOrientation finalOrientation = new FrameOrientation(worldFrame, trajectoryPointsOrientation.get(arrayIndex+1));
      FrameVector3D finalAngularVelocity = new FrameVector3D(worldFrame, trajectoryPointsAngularVelocity.get(arrayIndex+1));
      
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      HermiteCurveBasedOrientationTrajectoryGenerator traj1 = new HermiteCurveBasedOrientationTrajectoryGenerator("traj1", worldFrame, registry);
      traj1.setInitialConditions(initialOrientation, initialAngularVelocity);
      traj1.setFinalConditions(finalOrientation, finalAngularVelocity);
      traj1.setTrajectoryTime(localEndTime - localStartTime);
      traj1.setNumberOfRevolutions(0);
      traj1.initialize();
      
      traj1.compute(time - localStartTime);
      FrameOrientation interpolatedOrientation = new FrameOrientation(worldFrame);
      traj1.getOrientation(interpolatedOrientation);
      
      Quaternion orientation = new Quaternion(interpolatedOrientation.getQuaternion());
      
      return orientation;
   }
   
   public Vector3D getOrientationYawPitchRoll(double time)
   {
      Vector3D angularVelocity = new Vector3D();
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(getOrientation(time), angularVelocity);
      
      return angularVelocity;
   }
}
