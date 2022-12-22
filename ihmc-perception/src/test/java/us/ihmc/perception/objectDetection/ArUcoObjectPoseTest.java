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

import static us.ihmc.robotics.Assert.assertEquals;

public class ArUcoObjectPoseTest
{
   @Test
   public void testFramePoseTransformsArUco()
   {
      ArUcoObjectInfo arucoInfo = new ArUcoObjectInfo();
      ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
      ArUcoObject objectWithArUco;

      // add markers to add with their respective info
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
      assertEquals((float) objectPose.getTranslation().getX(), (float) arucoInfo.getObjectTranslation(0).getX());
      assertEquals((float) objectPose.getTranslation().getY(), (float) arucoInfo.getObjectTranslation(0).getY());
      assertEquals((float) objectPose.getTranslation().getZ(), (float) arucoInfo.getObjectTranslation(0).getZ());
      objectPose.changeFrame(ReferenceFrame.getWorldFrame()); // transform back to world frame

      ReferenceFrame objectFrame = objectWithArUco.getObjectFrame();
      markerPose.changeFrame(objectFrame); // transform marker pose in object frame
      LogTools.info("Marker: {}", markerPose);
      // check that marker pose in object frame equals to inverse transform marker to object
      assertEquals((float) markerPose.getTranslation().getX(), -(float) arucoInfo.getObjectTranslation(0).getX());
      assertEquals((float) markerPose.getTranslation().getY(), -(float) arucoInfo.getObjectTranslation(0).getY());
      assertEquals((float) markerPose.getTranslation().getZ(), -(float) arucoInfo.getObjectTranslation(0).getZ());
      markerPose.changeFrame(ReferenceFrame.getWorldFrame()); // transform back to world frame

      FramePose3D recordedGoalPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     new Point3D(0.878, -0.267,  0.608 ),
                                                     new Quaternion(0.677, -0.184,  0.158,  0.695));
      LogTools.info("Recorded Goal: {}", recordedGoalPose);
      recordedGoalPose.changeFrame(objectFrame);
      LogTools.info("Recorded Goal: {}", recordedGoalPose);

      FramePose3D goalPose = new FramePose3D(objectPose);
      RigidBodyTransform transformObjectToGoal = new RigidBodyTransform(); // new RigidBodyTransform(recordedGoalPose); // cannot create transform from pose like this probably
      recordedGoalPose.get(transformObjectToGoal); // pack transform object to goal from goal pose in object frame
//      transformObjectToGoal.invert();
//      goalPose.appendRotation(transformObjectToGoal.getRotation());
//      goalPose.appendTranslation(transformObjectToGoal.getTranslation());
      goalPose.applyInverseTransform(transformObjectToGoal);
      recordedGoalPose.changeFrame(ReferenceFrame.getWorldFrame());
      LogTools.info("Goal: {}", goalPose);
      assertEquals((float) goalPose.getTranslation().getX(), (float) recordedGoalPose.getX());
      assertEquals((float) goalPose.getTranslation().getY(), (float) recordedGoalPose.getY());
      assertEquals((float) goalPose.getTranslation().getZ(), (float) recordedGoalPose.getZ());
   }
}
