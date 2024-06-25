package us.ihmc.perception.sceneGraph;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

/**
 * An object that is currently detected or not currently detected,
 * as such with objects tracked via ArUco markers or YOLO.
 */
public class DetectableSceneNode extends SceneNode
{
   private final Map<UUID, InstantDetection> latestDetections = new HashMap<>();
   private final Map<UUID, ReferenceFrame> detectionFramesInWorld = new HashMap<>();
   private boolean currentlyDetected;

   public DetectableSceneNode(long id, String name, InstantDetection detection, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);
      addNewDetection(detection);
   }

   public DetectableSceneNode(long id, String name, List<InstantDetection> detections, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);
      setLatestDetections(detections);
   }

   public void update(SceneGraph sceneGraph)
   {
      latestDetections.forEach((persistentDetectionID, instantDetection) ->
      {
         ReferenceFrame detectionFrame = detectionFramesInWorld.get(persistentDetectionID);

         if (detectionFrame == null)
         {
            String frameName = "detection-" + persistentDetectionID.toString().substring(0, 5);
            detectionFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName,
                                                                                               ReferenceFrame.getWorldFrame(),
                                                                                               new RigidBodyTransform());
            detectionFramesInWorld.put(persistentDetectionID, detectionFrame);
         }

         detectionFrame.getTransformToParent().set(instantDetection.getPose());
      });
   }

   public void updateDetection(InstantDetection newDetection)
   {
      UUID newDetectionID = newDetection.getPersistentDetectionID();
      if (latestDetections.containsKey(newDetectionID) && newDetection.getDetectionTime().isAfter(latestDetections.get(newDetectionID).getDetectionTime()))
         latestDetections.replace(newDetection.getPersistentDetectionID(), newDetection);
   }

   public void setLatestDetections(Collection<? extends InstantDetection> newLatestDetections)
   {
      latestDetections.clear();
      for (InstantDetection detection : newLatestDetections)
      {
         addNewDetection(detection);
      }
   }

   /**
    * Add a new {@link InstantDetection} to the set of detections this node corresponds to
    * @param detection New {@link InstantDetection} being added.
    * @return True if the detection is added to the set, false if the set already contained the detection.
    */
   public boolean addNewDetection(InstantDetection detection)
   {
      if (!latestDetections.containsKey(detection.getPersistentDetectionID()))
      {
         String frameName = "detection-" + detection.getPersistentDetectionID().toString().substring(0, 5);
         ReferenceFrame detectionFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName,
                                                                                                           ReferenceFrame.getWorldFrame(),
                                                                                                           new RigidBodyTransform());
         detectionFramesInWorld.put(detection.getPersistentDetectionID(), detectionFrame);
      }

      return latestDetections.putIfAbsent(detection.getPersistentDetectionID(), detection) == null;
   }

   public boolean hasMatchingDetection(InstantDetection detectionToMatch)
   {
      return latestDetections.containsKey(detectionToMatch.getPersistentDetectionID());
   }

   public boolean hasMatchingDetection(PersistentDetection<? extends InstantDetection> detectionToMatch)
   {
      return latestDetections.containsKey(detectionToMatch.getID());
   }

   public Collection<? extends InstantDetection> getLatestDetections()
   {
      return latestDetections.values();
   }

   public InstantDetection getDetection(UUID detectionId)
   {
      return latestDetections.get(detectionId);
   }

   public Map<UUID, ReferenceFrame> getDetectionFramesInWorld()
   {
      return detectionFramesInWorld;
   }

   public ReferenceFrame getDetectionFrameInWorld(UUID detectionId)
   {
      return detectionFramesInWorld.get(detectionId);
   }

   public void setCurrentlyDetected(boolean currentlyDetected)
   {
      this.currentlyDetected = currentlyDetected;
   }

   public boolean getCurrentlyDetected()
   {
      return currentlyDetected;
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);

      DetectionManager detectionManager = sceneGraph.getDetectionManager();
      if (detectionManager != null)
      {
         latestDetections.forEach((id, instantDetection) ->
         {
            PersistentDetection<? extends InstantDetection> persistentDetection = detectionManager.getDetection(id, instantDetection.getClass());
            if (persistentDetection != null)
               persistentDetection.markForDeletion();
         });
      }
   }
}
