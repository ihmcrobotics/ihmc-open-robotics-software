package us.ihmc.perception.objectDetection;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ArUcoObject;
import us.ihmc.perception.ArUcoObjectInfo;
import us.ihmc.perception.OpenCVArUcoMarker;

import java.util.ArrayList;

import static org.junit.Assert.assertTrue;
import static us.ihmc.robotics.Assert.assertEquals;

public class ArUcoObjectPoseTest
{
   @Test
   public void testFramePoseTransformsArUco()
   {
      ArUcoObjectInfo arucoInfo = new ArUcoObjectInfo();
      ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
      ArUcoObject objectWithArUco;

      // add markers with their respective info
      for (int id : arucoInfo.getMarkersId())
      {
         markersToTrack.add(new OpenCVArUcoMarker(id, arucoInfo.getMarkerSize(id)));
      }

      // get a marker
      OpenCVArUcoMarker marker = markersToTrack.get(0);
      int objectId = marker.getId();

      // get object with attached aruco marker
      objectWithArUco = new ArUcoObject(objectId, arucoInfo);
      // get marker pose in world frame, in reality this would be detected by camera
      FramePose3DBasics markerPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     new Point3D(1.073, -0.146, 1.016),
                                                     new Quaternion(-0.002, 1.000, 0.001, 0.003));
      // create from this pose, the associated transform stored in objectWithArUco
      markerPose.get(objectWithArUco.getMarkerToWorld());
      objectWithArUco.update(); // update frame of the object
      objectWithArUco.computeObjectPose(markerPose); // compute object pose from marker pose
      LogTools.info("Marker: {}", markerPose);

      FramePose3D objectPose = objectWithArUco.getObjectPose();
      LogTools.info("Object: {}", objectPose);
      objectPose.changeFrame(objectWithArUco.getMarkerFrame()); // transform object pose in marker frame
      LogTools.info("Object: {}", objectPose);
      // check that object pose in marker frame equals to transform marker to object
      assertTrue((float) objectPose.getTranslation().getX() == (float) arucoInfo.getObjectTranslation(objectId).getX());
      assertTrue((float) objectPose.getTranslation().getY() == (float) arucoInfo.getObjectTranslation(objectId).getY());
      assertTrue((float) objectPose.getTranslation().getZ() == (float) arucoInfo.getObjectTranslation(objectId).getZ());
      assertTrue((float) Math.round(objectPose.getOrientation().getYaw() * 1000) / 1000 == (float) arucoInfo.getObjectYawPitchRoll(objectId).getYaw());
      assertTrue((float) Math.round(objectPose.getOrientation().getRoll() * 1000) / 1000 == (float) arucoInfo.getObjectYawPitchRoll(objectId).getRoll());
      assertTrue((float) Math.round(objectPose.getOrientation().getPitch() * 1000) / 1000 == (float) arucoInfo.getObjectYawPitchRoll(objectId).getPitch());
      objectPose.changeFrame(ReferenceFrame.getWorldFrame()); // transform back to world frame

      ReferenceFrame objectFrame = objectWithArUco.getObjectFrame();
      markerPose.changeFrame(objectFrame); // transform marker pose in object frame
      LogTools.info("Marker: {}", markerPose);
      // check that marker pose in object frame equals to inverse transform marker to object
      assertTrue((float) markerPose.getTranslation().getX() == -(float) arucoInfo.getObjectTranslation(objectId).getX());
      assertTrue((float) markerPose.getTranslation().getY() == -(float) arucoInfo.getObjectTranslation(objectId).getY());
      assertTrue((float) markerPose.getTranslation().getZ() == -(float) arucoInfo.getObjectTranslation(objectId).getZ());
      assertTrue((float) Math.round(markerPose.getOrientation().getYaw() * 1000) / 1000 == -(float) arucoInfo.getObjectYawPitchRoll(objectId).getYaw());
      assertTrue((float) Math.round(markerPose.getOrientation().getRoll() * 1000) / 1000 == -(float) arucoInfo.getObjectYawPitchRoll(objectId).getRoll());
      assertTrue((float) Math.round(markerPose.getOrientation().getPitch() * 1000) / 1000 == -(float) arucoInfo.getObjectYawPitchRoll(objectId).getPitch());
      markerPose.changeFrame(ReferenceFrame.getWorldFrame()); // transform back to world frame

      ///////////////////////////////////////////////////////////////////////////////
      System.out.println("--------------------------------------------------------------------------------------------");
      FramePose3D recordedGoalPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     new Point3D(0.878, -0.267, 0.608),
                                                     new Quaternion(0.677, -0.184, 0.158, 0.695));
      LogTools.info("Recorded Goal: {}", recordedGoalPose);
      FramePose3D recordedGoalPoseWorldFrame = new FramePose3D(recordedGoalPose);
      recordedGoalPose.changeFrame(objectFrame);
      FramePose3D recordedGoalPoseObjectFrame = new FramePose3D(recordedGoalPose);
      LogTools.info("Recorded Goal: {}", recordedGoalPose);

      RigidBodyTransform transformObjectToGoal = new RigidBodyTransform(); // new RigidBodyTransform(recordedGoalPose); // cannot create transform from pose like this probably
      recordedGoalPoseObjectFrame.get(transformObjectToGoal); // pack transformObjectToGoal, from recordedGoalPose in object frame
      LogTools.info("Translation Object to Goal: {}", transformObjectToGoal.getTranslation());
      LogTools.info("Object: {}", objectPose);
      //      objectPose.applyInverseTransform(transformObjectToGoal);
      objectPose.appendTransform(transformObjectToGoal);
      LogTools.info("Result: {}", objectPose);
      assertEquals((float) objectPose.getTranslation().getX(), (float) recordedGoalPoseWorldFrame.getX());
      assertEquals((float) objectPose.getTranslation().getY(), (float) recordedGoalPoseWorldFrame.getY());
      assertEquals((float) objectPose.getTranslation().getZ(), (float) recordedGoalPoseWorldFrame.getZ());
      assertEquals((float) objectPose.getOrientation().getX(), (float) recordedGoalPoseWorldFrame.getOrientation().getX());
      assertEquals((float) objectPose.getOrientation().getY(), (float) recordedGoalPoseWorldFrame.getOrientation().getY());
      assertEquals((float) objectPose.getOrientation().getZ(), (float) recordedGoalPoseWorldFrame.getOrientation().getZ());
      assertEquals((float) objectPose.getOrientation().getS(), (float) recordedGoalPoseWorldFrame.getOrientation().getS());
   }
}
