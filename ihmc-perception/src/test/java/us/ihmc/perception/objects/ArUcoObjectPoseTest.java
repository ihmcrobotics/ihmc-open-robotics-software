package us.ihmc.perception.objects;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.perception.OpenCVArUcoMarker;

import java.util.ArrayList;

import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertGeometricallyEquals;
import static us.ihmc.robotics.Assert.assertEquals;

public class ArUcoObjectPoseTest
{
   @Test
   public void framePoseTransformsArUcoTest()
   {
      ArUcoMarkerObjectInfo arucoInfo = new ArUcoMarkerObjectInfo();
      arucoInfo.load();
      ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
      ArUcoMarkerObject objectWithArUco;

      // add markers with their respective info
      for (int id : arucoInfo.getMarkersId())
      {
         markersToTrack.add(new OpenCVArUcoMarker(id, arucoInfo.getMarkerSize(id)));
      }

      // get a marker
      OpenCVArUcoMarker marker = markersToTrack.get(0);
      int objectId = marker.getId();

      // get object with attached aruco marker
      objectWithArUco = new ArUcoMarkerObject(objectId, arucoInfo);
      // get marker pose in world frame, in reality this would be detected by camera
      FramePose3DBasics markerPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     new Point3D(1.073, -0.146, 1.016),
                                                     new Quaternion(-0.002, 1.000, 0.001, 0.003));
      // create from this pose, the associated transform stored in objectWithArUco
      markerPose.get(objectWithArUco.getMarkerTransformToWorld());
      objectWithArUco.updateFrame(); // update frame of the object
      objectWithArUco.computeObjectPose(markerPose); // compute object pose from marker pose
      LogTools.info("Marker: {}", markerPose);

      FramePose3D objectPose = objectWithArUco.getObjectPose();
      LogTools.info("Object: {}", objectPose);
      objectPose.changeFrame(objectWithArUco.getMarkerFrame()); // transform object pose in marker frame
      LogTools.info("Object: {}", objectPose);
      YawPitchRoll objectYawPitchRoll = new YawPitchRoll();
      objectPose.getOrientation().get(objectYawPitchRoll);
      // check that object pose in marker frame equals to transform marker to object
      assertEquals(objectPose.getTranslation().getX(), arucoInfo.getMarkerTranslation(objectId).getX(), 1e-10);
      assertEquals(objectPose.getTranslation().getY(), arucoInfo.getMarkerTranslation(objectId).getY(), 1e-10);
      assertEquals(objectPose.getTranslation().getZ(), arucoInfo.getMarkerTranslation(objectId).getZ(), 1e-10);
      assertGeometricallyEquals(objectYawPitchRoll, arucoInfo.getMarkerYawPitchRoll(objectId), 1e-3);
      objectPose.changeFrame(ReferenceFrame.getWorldFrame()); // transform back to world frame

      ReferenceFrame objectFrame = objectWithArUco.getObjectFrame();
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

      RigidBodyTransform transformObjectToGoal = new RigidBodyTransform(); // cannot create transform from pose like this probably
      recordedGoalPoseObjectFrame.get(transformObjectToGoal); // pack transformObjectToGoal, from recordedGoalPose in object frame
      //      objectPose.applyInverseTransform(transformObjectToGoal);
      objectPose.appendTransform(transformObjectToGoal);
      LogTools.info("Result: {}", objectPose);
      assertGeometricallyEquals(objectPose,recordedGoalPoseWorldFrame,1e-10);
   }
}
