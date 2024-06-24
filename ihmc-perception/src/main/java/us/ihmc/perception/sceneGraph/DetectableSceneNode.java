package us.ihmc.perception.sceneGraph;

import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.crdt.CRDTInfo;

/**
 * An object that is currently detected or not currently detected,
 * as such with objects tracked via ArUco markers or YOLO.
 */
public class DetectableSceneNode extends SceneNode
{
   private List<PersistentDetection<? extends InstantDetection>> detections = new ArrayList<>();
   private boolean currentlyDetected;

   public DetectableSceneNode(long id, String name, PersistentDetection<? extends InstantDetection> detection, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);
      detections.add(detection);
   }

   public DetectableSceneNode(long id, String name, List<PersistentDetection<? extends InstantDetection>> detections, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);
      setDetections(detections);
   }

   public void update()
   {

   }

   public void setDetections(List<PersistentDetection<? extends InstantDetection>> detections)
   {
      this.detections = detections;
   }

   /**
    * Add a new {@link PersistentDetection} to the set of detections this node corresponds to
    * @param detection New {@link PersistentDetection} being added.
    * @return True if the detection is added to the set, false if the set already contained the detection.
    */
   public boolean addNewDetection(PersistentDetection<? extends InstantDetection> detection)
   {
      return detections.add(detection);
   }

   public boolean hasDetection(PersistentDetection<? extends InstantDetection> detection)
   {
      return detections.contains(detection);
   }

   public List<PersistentDetection<? extends InstantDetection>> getDetections()
   {
      return detections;
   }

   public PersistentDetection<? extends InstantDetection> getDetection(int detectionIndex)
   {
      return detections.get(detectionIndex);
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
   public void destroy()
   {
      super.destroy();

      detections.forEach(PersistentDetection::markForDeletion);
   }
}
