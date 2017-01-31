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
      
      System.out.println("Pelvis to Mocap Tranform: \n\n" + pelvisToMocapTransform);
      System.out.println("World to Mocap Transform: \n\n" + worldToMocapTransform);

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
      
      System.out.println("X axis in mocap frame: " + robotXAxisInMocapFrame);
      System.out.println("Y axis in mocap frame: " + robotYAxisInMocapFrame);
      System.out.println("Z axis in mocap frame: " + robotZAxisInMocapFrame);

      Matrix3d robotToMocapRotationMatrix = new Matrix3d();
      robotToMocapRotationMatrix.setColumn(0, robotXAxisInMocapFrame.getX(), robotXAxisInMocapFrame.getY(), robotXAxisInMocapFrame.getZ());
      robotToMocapRotationMatrix.setColumn(1, robotYAxisInMocapFrame.getX(), robotYAxisInMocapFrame.getY(), robotYAxisInMocapFrame.getZ());
      robotToMocapRotationMatrix.setColumn(2, robotZAxisInMocapFrame.getX(), robotZAxisInMocapFrame.getY(), robotZAxisInMocapFrame.getZ());

      System.out.println("Robot to Mocap rotation matrix: \n\n " + robotToMocapRotationMatrix);
      
      Vector3d marker2ToPlateOriginInMocapFrame = new Vector3d(markerPoint2InPelvisFrame);
      marker2ToPlateOriginInMocapFrame.negate();
      robotToMocapRotationMatrix.transform(marker2ToPlateOriginInMocapFrame);

      System.out.println("Marker 2 to plate origin: \n\n " + marker2ToPlateOriginInMocapFrame);

      Vector3d plateOriginToPelvisJointInMocapFrame = new Vector3d(markerPlateOriginInPelvisFrame);
      plateOriginToPelvisJointInMocapFrame.negate();
      robotToMocapRotationMatrix.transform(plateOriginToPelvisJointInMocapFrame);

      System.out.println("Plate origin to Pelvis Joint in Mocap: \n\n " + plateOriginToPelvisJointInMocapFrame);

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
