package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.robotSide.RobotSide;

public enum VRTrackedSegmentType
{
   LEFT_HAND("leftHand", RobotSide.LEFT, new Point3D(-0.0, 0.0, 0.350), new YawPitchRoll(0.0, -Math.PI / 2.0, 0.0), Double.NaN, Double.NaN),
   RIGHT_HAND("rightHand", RobotSide.RIGHT, new Point3D(-0.0, -0.0, 0.350), new YawPitchRoll(0.0, -Math.PI / 2.0, 0.0), Double.NaN, Double.NaN),
   LEFT_FOREARM("leftForeArm", RobotSide.LEFT, new Point3D(), new YawPitchRoll(-Math.PI / 2.0, 0.0, 0.0), 0.0, 1),
   RIGHT_FOREARM("rightForeArm", RobotSide.RIGHT, new Point3D(), new YawPitchRoll(Math.PI / 2.0, 0.0, 0.0), 0.0, 1),
   CHEST("chest", null, new Point3D(), new YawPitchRoll(), 0.0, 10);

   private String segmentName;
   private RobotSide robotSide;
   private Point3D trackerToSegmentTranslation;
   private YawPitchRoll trackerToSegmentRotation;
   private double positionWeight;
   private double orientationWeight;

   VRTrackedSegmentType(String segmentName, RobotSide robotSide, Point3D trackerToSegmentTranslation, YawPitchRoll trackerToSegmentRotation, double positionWeight, double orientationWeight)
   {
      this.segmentName = segmentName;
      this.robotSide = robotSide;
      this.trackerToSegmentRotation = trackerToSegmentRotation;
      this.trackerToSegmentTranslation = trackerToSegmentTranslation;
      this.positionWeight = positionWeight;
      this.orientationWeight = orientationWeight;
   }

   public String getSegmentName()
   {
      return segmentName;
   }

   public RobotSide getSegmentSide()
   {
      return robotSide;
   }

   public Point3D getTrackerToSegmentTranslation()
   {
      return trackerToSegmentTranslation;
   }

   public YawPitchRoll getTrackerToSegmentRotation()
   {
      return trackerToSegmentRotation;
   }

   public double getPositionWeight()
   {
      return positionWeight;
   }

   public double getOrientationWeight()
   {
      return orientationWeight;
   }
}