package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ThreePointsAverageReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -2092644496487794572L;

   private final FramePoint framePoint1;
   private final FramePoint framePoint2;
   private final FramePoint framePoint3;

   private final FrameVector vectorOrigin;
   private final FrameVector vectorFromCenterToPoint1;
   private final FrameVector vectorFromCenterToPoint2;
   private final FrameVector vectorFromCenterToPoint3;

   private final FrameVector normalVector;
   
   private final FramePoint averagePoint;
   private final FrameOrientation averageOrientation;

   private final FramePlane3d framePlane;
   
   //temporary variable
   private final Quat4d temporaryQuaternion = new Quat4d(0.0, 0.0, 0.0, 1.0);
   private final Vector3d temporaryVector = new Vector3d();
   
   public ThreePointsAverageReferenceFrame(String frameName, FramePoint p1, FramePoint p2, FramePoint p3, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);
      
      p1.checkReferenceFrameMatch(parentFrame);
      p2.checkReferenceFrameMatch(parentFrame);
      p3.checkReferenceFrameMatch(parentFrame);

      framePoint1 = p1;
      framePoint2 = p2;
      framePoint3 = p3;

      averagePoint = new FramePoint(parentFrame);
      averageOrientation = new FrameOrientation(parentFrame);
      
      framePlane = new FramePlane3d(parentFrame);
      
      vectorOrigin = new FrameVector(parentFrame, 1.0, 0.0, 0.0);
      vectorFromCenterToPoint1 = new FrameVector(parentFrame);
      vectorFromCenterToPoint2 = new FrameVector(parentFrame);
      vectorFromCenterToPoint3 = new FrameVector(parentFrame);
      
      normalVector = new FrameVector(parentFrame);
   }

   private void updateAveragePoint()
   {
      averagePoint.set(framePoint1);
      averagePoint.add(framePoint2);
      averagePoint.add(framePoint3);

      averagePoint.scale(1.0 / 3.0);
   }
   
   private void updateAverageOrientation()
   {
      framePlane.setPoints(framePoint1, framePoint2, framePoint3);
      
      //construct the 3 vectors going from center to p1, p2 and p3
      double tempX = framePoint1.getX() - averagePoint.getX();
      double tempY = framePoint1.getY() - averagePoint.getY();
      double tempZ = framePoint1.getZ() - averagePoint.getZ();
      vectorFromCenterToPoint1.set(tempX, tempY, tempZ);
      
      tempX = framePoint2.getX() - averagePoint.getX();
      tempY = framePoint2.getY() - averagePoint.getY();
      tempZ = framePoint2.getZ() - averagePoint.getZ();
      vectorFromCenterToPoint2.set(tempX, tempY, tempZ);
      
      tempX = framePoint3.getX() - averagePoint.getX();
      tempY = framePoint3.getY() - averagePoint.getY();
      tempZ = framePoint3.getZ() - averagePoint.getZ();
      vectorFromCenterToPoint3.set(tempX, tempY, tempZ);
      
      
      //calculate the average angle 
      double angleFromOriginTo1 =  vectorOrigin.angle(vectorFromCenterToPoint1);
      double angleFrom1To2 = vectorFromCenterToPoint1.angle(vectorFromCenterToPoint2);
      double angleFrom1To3 = vectorFromCenterToPoint1.angle(vectorFromCenterToPoint3);
      
      double averageAngle = AngleTools.computeAngleAverage(angleFrom1To2, angleFrom1To3);
      
      framePlane.getNormal(normalVector);
      if(normalVector.getZ() < 0.0)
         normalVector.negate();
      
      normalVector.get(temporaryVector);
      
      RotationTools.getQuaternionFromYawAndZNormal(angleFromOriginTo1 + averageAngle, temporaryVector, temporaryQuaternion);
      
      averageOrientation.set(temporaryQuaternion);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {      
      updateAveragePoint();
      updateAverageOrientation();
      
      averagePoint.get(temporaryVector);
      averageOrientation.getQuaternion(temporaryQuaternion);
      
      transformToParent.set(temporaryQuaternion, temporaryVector);
   }
}
