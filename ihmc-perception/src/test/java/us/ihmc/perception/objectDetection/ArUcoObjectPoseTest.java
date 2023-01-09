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
import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertGeometricallyEquals;
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
      assertEquals(objectPose.getTranslation().getX(), arucoInfo.getObjectTranslation(objectId).getX(), 1e-10);
      assertEquals(objectPose.getTranslation().getY(), arucoInfo.getObjectTranslation(objectId).getY(), 1e-10);
      assertEquals(objectPose.getTranslation().getZ(), arucoInfo.getObjectTranslation(objectId).getZ(), 1e-10);
      assertEquals(objectPose.getOrientation().getYaw(), arucoInfo.getObjectYawPitchRoll(objectId).getYaw(), 1e-10);
      assertEquals(objectPose.getOrientation().getRoll(), arucoInfo.getObjectYawPitchRoll(objectId).getRoll(), 1e-10);
      assertEquals(objectPose.getOrientation().getPitch(), arucoInfo.getObjectYawPitchRoll(objectId).getPitch(), 1e-10);
      objectPose.changeFrame(ReferenceFrame.getWorldFrame()); // transform back to world frame

      ReferenceFrame objectFrame = objectWithArUco.getObjectFrame();
      markerPose.changeFrame(objectFrame); // transform marker pose in object frame
      LogTools.info("Marker: {}", markerPose);
      // check that marker pose in object frame equals to inverse transform marker to object
      assertEquals(markerPose.getTranslation().getX(), -arucoInfo.getObjectTranslation(objectId).getX(), 1e-10);
      assertEquals(markerPose.getTranslation().getY(), -arucoInfo.getObjectTranslation(objectId).getY(), 1e-10);
      assertEquals(markerPose.getTranslation().getZ(), -arucoInfo.getObjectTranslation(objectId).getZ(), 1e-10);
      assertEquals(markerPose.getOrientation().getYaw(), -arucoInfo.getObjectYawPitchRoll(objectId).getYaw(), 1e-10);
      assertEquals(markerPose.getOrientation().getRoll(), -arucoInfo.getObjectYawPitchRoll(objectId).getRoll(), 1e-10);
      assertEquals(markerPose.getOrientation().getPitch(), -arucoInfo.getObjectYawPitchRoll(objectId).getPitch(), 1e-10);
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
      //      objectPose.applyInverseTransform(transformObjectToGoal);
      objectPose.appendTransform(transformObjectToGoal);
      LogTools.info("Result: {}", objectPose);
      assertGeometricallyEquals(objectPose,recordedGoalPoseWorldFrame,1e-10);
   }
}
