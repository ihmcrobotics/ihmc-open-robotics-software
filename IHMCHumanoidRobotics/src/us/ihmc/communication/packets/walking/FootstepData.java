package us.ihmc.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.TransformableDataObject;
import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

@ClassDocumentation("This message specifies the position, orientation and side (left or right) of a desired footstep in\n"
                                  + "world frame")
public class FootstepData extends IHMCRosApiPacket<FootstepData> implements TransformableDataObject<FootstepData>
{
   // Must be public for efficient serialization.
   public RobotSide robotSide;
   public Point3d location;
   public Quat4d orientation;
   
   @FieldDocumentation("predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n"
                                     + "the world. A value of null will default to using the entire foot. Contact points should be specified\n"
                                     + "in foot sole frame, where the origin is at the center of the foot. Order of the points does not matter.\n"
                                     + "For example: to tell the controller to use the entire foot, the predicted contact points would be:\n"
                                     + "predicted_contact_points:\n"
                                     + "- {x: 0.5 * foot_length, y: -0.5 * toe_width}\n"
                                     + "- {x: 0.5 * foot_length, y: 0.5 * toe_width}\n"
                                     + "- {x: -0.5 * foot_length, y: -0.5 * heel_width}\n"
                                     + "- {x: -0.5 * foot_length, y: 0.5 * heel_width}\n")
   public ArrayList<Point2d> predictedContactPoints;

   @FieldDocumentation("This contains information on what the swing trajectory should be for each step. Recomended is to default to basic.\n")
   public TrajectoryType trajectoryType = TrajectoryType.DEFAULT;

   @FieldDocumentation("Contains information on how high the robot should step. This affects only basic and obstacle clearance trajectories."
         + "Recommended values are between 0.1 (default) and 0.25.\n")
   public double swingHeight = 0;

   public FootstepData()
   {
      // Must have null constructor for efficient serialization.
   }

   public FootstepData(RobotSide robotSide, Point3d location, Quat4d orientation)
   {
      this(robotSide, location, orientation, null);
   }

   public FootstepData(RobotSide robotSide, Point3d location, Quat4d orientation, ArrayList<Point2d> predictedContactPoints){
      this(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }
   public FootstepData(RobotSide robotSide, Point3d location, Quat4d orientation, TrajectoryType trajectoryType, double swingHeight)
   {
      this(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }


   public FootstepData(RobotSide robotSide, Point3d location, Quat4d orientation, ArrayList<Point2d> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      this.robotSide = robotSide;
      this.location = location;
      this.orientation = orientation;
      if (predictedContactPoints != null && predictedContactPoints.size() == 0)
         this.predictedContactPoints = null;
      else
         this.predictedContactPoints = predictedContactPoints;
      this.trajectoryType = trajectoryType;
      this.swingHeight = swingHeight;
   }

   public FootstepData(FootstepData footstepData)
   {
      this.robotSide = footstepData.robotSide;
      this.location = new Point3d(footstepData.location);
      this.orientation = new Quat4d(footstepData.orientation);
      RotationFunctions.checkUnitLength(this.orientation);
      if (footstepData.predictedContactPoints == null || footstepData.predictedContactPoints.isEmpty()){
         this.predictedContactPoints = null;
      }else{
         this.predictedContactPoints = new ArrayList<>();
         for (Point2d contactPoint : footstepData.predictedContactPoints){
            this.predictedContactPoints.add(new Point2d(contactPoint));
         }
      }
      this.trajectoryType = footstepData.trajectoryType;
      this.swingHeight = footstepData.swingHeight;
   }

   public FootstepData clone(){
      return new FootstepData(this);
   }

   public FootstepData(Footstep footstep){
      robotSide = footstep.getRobotSide();
      location = new Point3d();
      orientation = new Quat4d();
      footstep.getPositionInWorldFrame(location);
      footstep.getOrientationInWorldFrame(orientation);

      List<Point2d> footstepContactPoints = footstep.getPredictedContactPoints();
      if (footstepContactPoints != null){
         if (predictedContactPoints == null)
         {
            predictedContactPoints = new ArrayList<>();
         }else{
            predictedContactPoints.clear();
         }
         for (Point2d contactPoint: footstepContactPoints){
               predictedContactPoints.add((Point2d)contactPoint.clone());
         }
      }else{
         predictedContactPoints = null;
      }
      trajectoryType = footstep.trajectoryType;
      swingHeight = footstep.swingHeight;
   }

   public void setPredictedContactPoints(ArrayList<Point2d> predictedContactPoints)
   {
      if (predictedContactPoints != null && predictedContactPoints.size() == 0)
         throw new RuntimeException("Cannot have an empty list of contact points in FootstepData. Should be null to use the default controller contact points.");
      this.predictedContactPoints = predictedContactPoints;
   }
   
   public ArrayList<Point2d> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }
   
   public Point3d getLocation()
   {
      return location;
   }

   public void getLocation(Point3d locationToPack)
   {
      locationToPack.set(location);
   }

   public void setLocation(Point3d location)
   {
      this.location.set(location);
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public void getOrientation(Quat4d orientationToPack)
   {
      orientationToPack.set(this.orientation);
   }

   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
      RotationFunctions.checkUnitLength(this.orientation);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }


   public double getSwingHeight()
   {
      return swingHeight;
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
      if (trajectoryType == TrajectoryType.DEFAULT)
         trajectoryType = TrajectoryType.BASIC;

   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public String toString()
   {
      String ret = "";

      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), this.orientation);
      double[] ypr = frameOrientation.getYawPitchRoll();
      ret = location.toString();
      ret += ", YawPitchRoll = " + Arrays.toString(ypr) + "\n";
      ret += "Predicted Contact Points: ";
      if(predictedContactPoints != null)
      {
         ret += "size = " + predictedContactPoints.size() + "\n";
      }
      else
      {
         ret += "null";
      }

      return ret;
   }

   public boolean epsilonEquals(FootstepData footstepData, double epsilon)
   {
      boolean robotSideEquals = robotSide == footstepData.robotSide;
      boolean locationEquals = location.epsilonEquals(footstepData.location, epsilon);

      boolean orientationEquals = orientation.epsilonEquals(footstepData.orientation, epsilon);
      if (!orientationEquals)
      {
         Quat4d temp = new Quat4d();
         temp.negate(orientation);
         orientationEquals = temp.epsilonEquals(footstepData.orientation, epsilon);
      }
      
      boolean contactPointsEqual = true;
      
      if ((this.predictedContactPoints == null) && (footstepData.predictedContactPoints != null)) contactPointsEqual = false;
      else if ((this.predictedContactPoints != null) && (footstepData.predictedContactPoints == null)) contactPointsEqual = false;
      else if (this.predictedContactPoints != null)
      {
         int size = predictedContactPoints.size();
         if (size != footstepData.predictedContactPoints.size()) contactPointsEqual = false;
         else 
         {
            for (int i=0; i<size; i++)
            {
               Point2d pointOne = predictedContactPoints.get(i);
               Point2d pointTwo = footstepData.predictedContactPoints.get(i);
               
               if (!(pointOne.distanceSquared(pointTwo) < 1e-7)) contactPointsEqual = false;
            }
         }
      }

      return robotSideEquals && locationEquals && orientationEquals && contactPointsEqual;
   }

   public FootstepData transform(RigidBodyTransform transform)
   {
      FootstepData ret = this.clone();

      // Point3d location;
      ret.location = TransformTools.getTransformedPoint(this.getLocation(), transform);

      // Quat4d orientation;
      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);
      return ret;
   }

   public FootstepData(Random random)
   {
      TrajectoryType[] trajectoryTypes = TrajectoryType.values();
      int randomOrdinal = random.nextInt(trajectoryTypes.length);
      
      this.robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;;
      this.location = RandomTools.generateRandomPointWithEdgeCases(random, 0.05);
      this.orientation = RandomTools.generateRandomQuaternion(random);
      int numberOfPredictedContactPoints = random.nextInt(10);
      if(numberOfPredictedContactPoints == 0)
         predictedContactPoints = null;
      else
      {
         this.predictedContactPoints = new ArrayList<>();
         for(int i = 0; i < numberOfPredictedContactPoints; i++)
         {
            predictedContactPoints.add(new Point2d(random.nextDouble(), random.nextDouble()));
         }
      }
      this.trajectoryType = trajectoryTypes[randomOrdinal];
      this.swingHeight = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.05);
   }
}
