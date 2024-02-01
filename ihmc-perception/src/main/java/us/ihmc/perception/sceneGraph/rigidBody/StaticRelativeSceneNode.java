package us.ihmc.perception.sceneGraph.rigidBody;

import gnu.trove.map.TLongObjectMap;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.SceneNode;

/**
 * This node stays in the same spot relative to where a parent scene node
 * at the time it is seen up close.
 * <p>
 * The whole point of this is so we don't have to put markers on everything,
 * especially things that don't move.
 */
public class StaticRelativeSceneNode extends PredefinedRigidBodySceneNode
{
   /**
    * We don't want to lock in the static pose until we are close enough
    * for it to matter and also to get higher accuracy.
    */
   private double distanceToDisableTracking;
   private double currentDistance = Double.NaN;
   private transient final FramePose3D staticRelativeSceneNodePose = new FramePose3D();

   public StaticRelativeSceneNode(long id,
                                  String name,
                                  TLongObjectMap<SceneNode> sceneGraphIDToNodeMap,
                                  long initialParentNodeID,
                                  RigidBodyTransformReadOnly initialTransformToParent,
                                  String visualModelFilePath,
                                  RigidBodyTransform visualModelToNodeFrameTransform,
                                  double distanceToDisableTracking)
   {
      super(id, name, sceneGraphIDToNodeMap, initialParentNodeID, initialTransformToParent, visualModelFilePath, visualModelToNodeFrameTransform);

      this.distanceToDisableTracking = distanceToDisableTracking;
   }

   /**
    * Should only happen on the robot, not the UI.
    */
   public void updateTrackingState(ReferenceFrame sensorFrame, SceneGraphModificationQueue modificationQueue)
   {
      staticRelativeSceneNodePose.setToZero(getNodeFrame());
      staticRelativeSceneNodePose.setFromReferenceFrame(sensorFrame);
      currentDistance = staticRelativeSceneNodePose.getPosition().distanceFromOrigin();

      if (currentDistance <= getDistanceToDisableTracking() && getTrackingInitialParent())
      {
         LogTools.warn("{}: Disabling tracking initial parent", getName());
         setTrackInitialParent(false, modificationQueue);
      }
   }

   public void setDistanceToDisableTracking(double distanceToDisableTracking)
   {
      this.distanceToDisableTracking = distanceToDisableTracking;
   }

   public double getDistanceToDisableTracking()
   {
      return distanceToDisableTracking;
   }

   public void setCurrentDistance(double currentDistance)
   {
      this.currentDistance = currentDistance;
   }

   public double getCurrentDistance()
   {
      return currentDistance;
   }
}
