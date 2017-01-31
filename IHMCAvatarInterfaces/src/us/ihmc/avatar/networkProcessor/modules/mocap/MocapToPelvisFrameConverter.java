package us.ihmc.avatar.networkProcessor.modules.mocap;

import optiTrack.MocapMarker;
import optiTrack.MocapRigidBody;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

public class MocapToPelvisFrameConverter
{
   private boolean initialized = false;

   private ReferenceFrame mocapFrame = null;

   private static final Vector3d markerPlateOriginInPelvisFrame = new Vector3d(0.1719, 0.0, 0.11324);
   private static final double ballRadius = 0.006;

   private static final Vector3d markerPoint1OffsetInPelvisFrame = new Vector3d(0.02 + ballRadius, - 0.04387, 0.0213);
   private static final Vector3d markerPoint2OffsetInPelvisFrame = new Vector3d(0.005 + ballRadius, 0.04445, 0.0213);
   private static final Vector3d markerPoint3OffsetInPelvisFrame = new Vector3d(0.005 + ballRadius, 0.04445, -0.0381);
   private static final Vector3d markerPoint4OffsetInPelvisFrame = new Vector3d(0.005 + ballRadius, - 0.04387, -0.0381);

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

      initialized = true;

      mocapFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("mocapFrame", ReferenceFrame.getWorldFrame(), worldToMocapTransform);
   }

   private RigidBodyTransform computePelvisToMocapTransform(ArrayList<MocapMarker> mocapMarkers)
   {
      MocapMarker marker1 = mocapMarkers.get(0);
      MocapMarker marker2 = mocapMarkers.get(1);
      MocapMarker marker3 = mocapMarkers.get(2);
      MocapMarker marker4 = mocapMarkers.get(3);

      Vector3d robotXAxisInMocapFrame = new Vector3d();
      Vector3d robotYAxisInMocapFrame = new Vector3d();
      Vector3d robotZAxisInMocapFrame = new Vector3d();

      robotYAxisInMocapFrame.sub(marker3.getPosition(), marker4.getPosition());
      robotZAxisInMocapFrame.sub(marker2.getPosition(), marker3.getPosition());
      robotXAxisInMocapFrame.cross(robotYAxisInMocapFrame, robotZAxisInMocapFrame);

      robotXAxisInMocapFrame.normalize();
      robotYAxisInMocapFrame.normalize();
      robotZAxisInMocapFrame.normalize();

      Matrix3d robotToMocapRotationMatrix = new Matrix3d();
      robotToMocapRotationMatrix.setColumn(0, robotXAxisInMocapFrame.getX(), robotXAxisInMocapFrame.getY(), robotXAxisInMocapFrame.getZ());
      robotToMocapRotationMatrix.setColumn(1, robotYAxisInMocapFrame.getX(), robotYAxisInMocapFrame.getY(), robotYAxisInMocapFrame.getZ());
      robotToMocapRotationMatrix.setColumn(2, robotZAxisInMocapFrame.getX(), robotZAxisInMocapFrame.getY(), robotZAxisInMocapFrame.getZ());

      Vector3d marker2ToPlateOriginInMocapFrame = new Vector3d(markerPoint2InPelvisFrame);
      marker2ToPlateOriginInMocapFrame.negate();
      robotToMocapRotationMatrix.transform(marker2ToPlateOriginInMocapFrame);

      Vector3d plateOriginToPelvisJointInMocapFrame = new Vector3d(markerPlateOriginInPelvisFrame);
      plateOriginToPelvisJointInMocapFrame.negate();
      robotToMocapRotationMatrix.transform(plateOriginToPelvisJointInMocapFrame);

      Vector3d pelvisJointInMocapFrame = new Vector3d(marker2.getPosition());
      pelvisJointInMocapFrame.add(marker2ToPlateOriginInMocapFrame);
      pelvisJointInMocapFrame.add(plateOriginToPelvisJointInMocapFrame);

      RigidBodyTransform pelvisToMocapTransform = new RigidBodyTransform();
      pelvisToMocapTransform.set(robotToMocapRotationMatrix, pelvisJointInMocapFrame);
      pelvisToMocapTransform.invert();

      return pelvisToMocapTransform;
   }

   public void getPelvisTransformFromMocapRigidBody(MocapRigidBody mocapRigidBody, RigidBodyTransform pelvisTransform)
   {
      ArrayList<MocapMarker> mocapMarkers = mocapRigidBody.getListOfAssociatedMarkers();
      if(mocapMarkers.size() != 4)
      {
         throw new RuntimeException("Rigid body " + mocapRigidBody.getId() + " should have 4 markers, but it has " + mocapMarkers.size());
      }

      RigidBodyTransform pelvisToMocapTransform = computePelvisToMocapTransform(mocapMarkers);
      FramePose pelvisPose = new FramePose(mocapFrame, pelvisToMocapTransform);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPose.getRigidBodyTransform(pelvisTransform);
   }
}
