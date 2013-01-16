package us.ihmc.commonWalkingControlModules.controlModules;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LeeGoswamiGroundReactionWrenchDistributorTest
{

   @Test
   public void testSimpleWrenchDistribution()
   {
      ReferenceFrame centerOfMassFrame = ReferenceFrame.getWorldFrame();
      FrameVector gravitationalAcceleration = new FrameVector(centerOfMassFrame, 0.0, 0.0, -9.81);
      double mass = 100.0;
      int nSupportVectors = 2;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      
      LeeGoswamiGroundReactionWrenchDistributor distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, gravitationalAcceleration, mass, nSupportVectors, parentRegistry);
  
      double coefficientOfFriction = 1.0;
      double rotationalCoefficientOfFriction = 0.5;
      double footLength = 0.3;
      double footWidth = 0.15;
      Point3d leftMidfootLocation = new Point3d(0.0, 0.5, 0.0);
      SimplePlaneContactState leftFootContactState = new SimplePlaneContactState(footLength, footWidth, leftMidfootLocation);
      distributor.addContact(leftFootContactState, coefficientOfFriction, rotationalCoefficientOfFriction);
      
      Point3d rightMidfootLocation = new Point3d(0.0, -0.5, 0.0);
      SimplePlaneContactState rightFootContactState = new SimplePlaneContactState(footLength, footWidth, rightMidfootLocation);
      distributor.addContact(rightFootContactState, coefficientOfFriction, rotationalCoefficientOfFriction);
   
      Vector3d linearPart = new Vector3d();
      Vector3d angularPart = new Vector3d();
      
      SpatialForceVector groundReactionWrench = new SpatialForceVector(ReferenceFrame.getWorldFrame(), linearPart, angularPart);
      distributor.solve(groundReactionWrench);
      
      FrameVector leftForce = distributor.getForce(leftFootContactState);
      FrameVector rightForce = distributor.getForce(rightFootContactState);
      
      double epsilon = 1e-7;
      FrameVector expectedLeftForce = new FrameVector(gravitationalAcceleration);
      expectedLeftForce.scale(-0.5 * mass);
      assertTrue(leftForce.epsilonEquals(expectedLeftForce, epsilon));
      
      FrameVector expectedRightForce = new FrameVector(gravitationalAcceleration);
      expectedRightForce.scale(-0.5 * mass);
      assertTrue(rightForce.epsilonEquals(expectedRightForce, epsilon)); 
   }
   
   
   private static class SimplePlaneContactState implements PlaneContactState
   {
      private final ArrayList<FramePoint> contactPoints;
      private final ArrayList<FramePoint2d> contactPoints2d;
      
      public SimplePlaneContactState(double footLength, double footWidth, Point3d midfootLocation)
      {
         contactPoints = new ArrayList<FramePoint>();
         contactPoints2d = new ArrayList<FramePoint2d>();
         
         Point3d frontLeft = new Point3d(midfootLocation);
         frontLeft.setX(frontLeft.getX() + footLength/2.0);
         frontLeft.setY(frontLeft.getY() + footWidth/2.0);
         
         Point3d frontRight = new Point3d(midfootLocation);
         frontLeft.setX(frontLeft.getX() + footLength/2.0);
         frontLeft.setY(frontLeft.getY() - footWidth/2.0);
         
         Point3d backLeft = new Point3d(midfootLocation);
         frontLeft.setX(frontLeft.getX() - footLength/2.0);
         frontLeft.setY(frontLeft.getY() + footWidth/2.0);
         
         Point3d backRight = new Point3d(midfootLocation);
         frontLeft.setX(frontLeft.getX() - footLength/2.0);
         frontLeft.setY(frontLeft.getY() - footWidth/2.0);
         
         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), frontLeft));
         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), frontRight));
         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), backLeft));
         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), backRight));
         
         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(frontLeft)));
         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(frontRight)));
         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(backLeft)));
         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(backRight)));
         
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
      
      
   };

}
