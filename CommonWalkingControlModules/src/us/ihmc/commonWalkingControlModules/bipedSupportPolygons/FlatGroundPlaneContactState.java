package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class FlatGroundPlaneContactState implements PlaneContactState
{
   private final ArrayList<FramePoint> contactPoints;
   private final ArrayList<FramePoint2d> contactPoints2d;
   private final double coefficientOfFriction;

   public static FlatGroundPlaneContactState createRandomFlatGroundContactState(Random random, boolean leftSide, double coefficientOfFriction)
   {
      double footLength = RandomTools.generateRandomDouble(random, 0.1, 0.3);
      double footWidth = RandomTools.generateRandomDouble(random, 0.1, 0.2);

      Point3d midfootLocation = RandomTools.generateRandomPoint(random, -1.0, 0.3, 0.0, 1.0, 1.0, 0.0);
      midfootLocation.setZ(0.0);

      if (!leftSide)
         midfootLocation.setY(-midfootLocation.getY());

      FlatGroundPlaneContactState flatGroundPlaneContactState = new FlatGroundPlaneContactState(footLength, footWidth, midfootLocation, coefficientOfFriction);

      return flatGroundPlaneContactState;
   }

   public FlatGroundPlaneContactState(double[][] contactPointLocations, double coefficientOfFriction)
   {
      contactPoints = new ArrayList<FramePoint>();
      contactPoints2d = new ArrayList<FramePoint2d>();

      for (double[] contactPointLocation : contactPointLocations)
      {
         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), contactPointLocation[0], contactPointLocation[1], 0.0));
         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), contactPointLocation[0], contactPointLocation[1]));
      }
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public FlatGroundPlaneContactState(double footLength, double footWidth, Point3d midfootLocation, double coefficientOfFriction)
   {
      contactPoints = new ArrayList<FramePoint>();
      contactPoints2d = new ArrayList<FramePoint2d>();

      Point3d frontLeft = new Point3d(midfootLocation);
      frontLeft.setX(frontLeft.getX() + footLength / 2.0);
      frontLeft.setY(frontLeft.getY() + footWidth / 2.0);

      Point3d frontRight = new Point3d(midfootLocation);
      frontRight.setX(frontRight.getX() + footLength / 2.0);
      frontRight.setY(frontRight.getY() - footWidth / 2.0);

      Point3d backLeft = new Point3d(midfootLocation);
      backLeft.setX(backLeft.getX() - footLength / 2.0);
      backLeft.setY(backLeft.getY() + footWidth / 2.0);

      Point3d backRight = new Point3d(midfootLocation);
      backRight.setX(backRight.getX() - footLength / 2.0);
      backRight.setY(backRight.getY() - footWidth / 2.0);

      contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), frontLeft));
      contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), frontRight));
      contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), backRight));
      contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), backLeft));

      contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(frontLeft)));
      contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(frontRight)));
      contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(backRight)));
      contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(backLeft)));

      this.coefficientOfFriction = coefficientOfFriction;
   }

   private Point2d projectToXY(Point3d point)
   {
      return new Point2d(point.getX(), point.getY());
   }

   public boolean inContact()
   {
      return true;
   }

   public List<FramePoint> getContactPoints()
   {
      return contactPoints;
   }

   public List<FramePoint2d> getContactPoints2d()
   {
      return contactPoints2d;
   }

   public ReferenceFrame getBodyFrame()
   {
      return ReferenceFrame.getWorldFrame();
   }

   public ReferenceFrame getPlaneFrame()
   {
      return ReferenceFrame.getWorldFrame();
   }


   public String toString()
   {
      return contactPoints2d.toString();
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

}
