package us.ihmc.avatar.networkProcessor.modules.mocap;

import optiTrack.MocapMarker;
import optiTrack.MocapRigidBody;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;


// SMcCrory 2/1/2017 - this module is specific to localization tests performed on Atlas given a certain marker configuration

public class MocapToPelvisFrameConverter
{
   private boolean initialized = false;

   private ReferenceFrame mocapFrame = null;

   private static final Vector3d markerPlateOriginInPelvisFrame = new Vector3d(0.1719, 0.0, 0.11324);
   private static final double ballRadius = 0.006;

   private static final Vector3d markerPoint1OffsetInPelvisFrame = new Vector3d(0.02 + ballRadius, - 0.04387, 0.0213);
   private static final Vector3d markerPoint2OffsetInPelvisFrame = new Vector3d(0.005 + ballRadius, 0.04445, 0.0213);
   private static final Vector3d markerPoint3OffsetInPelvisFrame = new Vector3d(0.005 + ballRadius, - 0.04387, -0.0381);
   private static final Vector3d markerPoint4OffsetInPelvisFrame = new Vector3d(0.005 + ballRadius, 0.04445, -0.0381);

   private static final Vector3d markerPoint1InPelvisFrame = new Vector3d();
   private static final Vector3d markerPoint2InPelvisFrame = new Vector3d();
   private static final Vector3d markerPoint3InPelvisFrame = new Vector3d();
   private static final Vector3d markerPoint4InPelvisFrame = new Vector3d();

   static
   {
      markerPoint1InPelvisFrame.add(markerPlateOriginInPelvisFrame, markerPoint1OffsetInPelvisFrame);
      markerPoint2InPelvisFrame.add(markerPlateOriginInPelvisFrame, markerPoint2OffsetInPelvisFrame);
      markerPoint3InPelvisFrame.add(markerPlateOriginInPelvisFrame, markerPoint3OffsetInPelvisFrame);
      markerPoint4InPelvisFrame.add(markerPlateOriginInPelvisFrame, markerPoint4OffsetInPelvisFrame);
   }

   public MocapToPelvisFrameConverter()
   {

   }

   public boolean isInitialized()
   {
      return initialized;
   }

   public void initialize(ReferenceFrame pelvisFrame, MocapRigidBody markerRigidBody)
   {
      ArrayList<MocapMarker> mocapMarkers = markerRigidBody.getListOfAssociatedMarkers();
      if(mocapMarkers.size() != 4)
      {
         throw new RuntimeException("Rigid body " + markerRigidBody.getId() + " should have 4 markers, but it has " + mocapMarkers.size());
      }

      RigidBodyTransform pelvisToMocapTransform = computePelvisToMocapTransform(mocapMarkers);
      RigidBodyTransform worldToPelvisTransform = pelvisFrame.getTransformToWorldFrame();
      worldToPelvisTransform.invert();

      RigidBodyTransform worldToMocapTransform = new RigidBodyTransform();
      worldToMocapTransform.multiply(pelvisToMocapTransform);
      worldToMocapTransform.multiply(worldToPelvisTransform);

      mocapFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("mocapFrame", ReferenceFrame.getWorldFrame(), worldToMocapTransform);
      initialized = true;
   }

   private RigidBodyTransform computePelvisToMocapTransform(ArrayList<MocapMarker> mocapMarkers)
   {
      MocapMarker marker1 = null;
      MocapMarker marker2 = null;
      MocapMarker marker3 = null;
      MocapMarker marker4 = null;
      
      for(MocapMarker mocapMarker : mocapMarkers)
      {
         if(mocapMarker.getId() == 1)
            marker1 = mocapMarker;
         else if(mocapMarker.getId() == 2)
            marker2 = mocapMarker;
         else if(mocapMarker.getId() == 3)
            marker3 = mocapMarker;
         else if(mocapMarker.getId() == 4)
            marker4 = mocapMarker;
      }

      Vector3d robotXAxisInMocapFrame = new Vector3d();
      Vector3d robotYAxisInMocapFrame = new Vector3d();
      Vector3d robotZAxisInMocapFrame = new Vector3d();

      robotYAxisInMocapFrame.sub(marker4.getPosition(), marker3.getPosition());
      robotZAxisInMocapFrame.sub(marker2.getPosition(), marker4.getPosition());
      robotXAxisInMocapFrame.cross(robotYAxisInMocapFrame, robotZAxisInMocapFrame);

      robotXAxisInMocapFrame.normalize();
      robotYAxisInMocapFrame.normalize();
      robotZAxisInMocapFrame.normalize();

      Matrix3d robotToMocapRotationMatrix = new Matrix3d();
      robotToMocapRotationMatrix.setColumn(0, robotXAxisInMocapFrame);
      robotToMocapRotationMatrix.setColumn(1, robotYAxisInMocapFrame);
      robotToMocapRotationMatrix.setColumn(2, robotZAxisInMocapFrame);

      Vector3d marker2ToPelvisInMocapFrame = new Vector3d(markerPoint2InPelvisFrame);
      marker2ToPelvisInMocapFrame.negate();
      robotToMocapRotationMatrix.transform(marker2ToPelvisInMocapFrame);

      Vector3d plateOriginToPelvisJointInMocapFrame = new Vector3d(markerPlateOriginInPelvisFrame);
      plateOriginToPelvisJointInMocapFrame.negate();
      robotToMocapRotationMatrix.transform(plateOriginToPelvisJointInMocapFrame);

      Vector3d pelvisJointInMocapFrame = new Vector3d(marker2.getPosition());
      pelvisJointInMocapFrame.add(marker2ToPelvisInMocapFrame);

      RigidBodyTransform pelvisToMocapTransform = new RigidBodyTransform(robotToMocapRotationMatrix, pelvisJointInMocapFrame);
      return pelvisToMocapTransform;
   }

   public void computePelvisToWorldTransform(MocapRigidBody mocapRigidBody, RigidBodyTransform pelvisToWorldTransformToPack)
   {
      ArrayList<MocapMarker> mocapMarkers = mocapRigidBody.getListOfAssociatedMarkers();
      if(mocapMarkers.size() != 4)
      {
         throw new RuntimeException("Rigid body " + mocapRigidBody.getId() + " should have 4 markers, but it has " + mocapMarkers.size());
      }

      RigidBodyTransform pelvisToMocapTransform = computePelvisToMocapTransform(mocapMarkers);
      FramePose pelvisPose = new FramePose(mocapFrame, pelvisToMocapTransform);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPose.getRigidBodyTransform(pelvisToWorldTransformToPack);
   }
   
   public static Vector3d getMarkerOffsetInPelvisFrame(int markerId)
   {
      switch(markerId)
      {
      case 1:
         return markerPoint1InPelvisFrame;
      case 2:
         return markerPoint2InPelvisFrame;
      case 3:
         return markerPoint3InPelvisFrame;
      case 4:
         return markerPoint4InPelvisFrame;
      default:
         return null;
      }
   }
}
