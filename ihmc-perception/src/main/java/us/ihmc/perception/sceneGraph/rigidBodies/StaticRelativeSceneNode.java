package us.ihmc.perception.sceneGraph.rigidBodies;

import gnu.trove.map.TLongObjectMap;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * This node stays in the same spot relative to where a parent scene node
 * at the time it is seen up close.
 *
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

   /** Should only happen on the robot, not the UI. */
   public void updateTrackingState(ReferenceFrame sensorFrame, SceneGraphModificationQueue modificationQueue)
   {
      if (!isFrozenFromModification())
      {
         staticRelativeSceneNodePose.setToZero(getNodeFrame());
         staticRelativeSceneNodePose.setFromReferenceFrame(sensorFrame);
         double updatedCurrentDistance = staticRelativeSceneNodePose.getPosition().distanceFromOrigin();
         if (!MathTools.epsilonEquals(currentDistance, updatedCurrentDistance, 0.1) && updatedCurrentDistance < currentDistance)
         {

            LogTools.info(("""
                  Type: %s
                  Old distance: %.2f
                  New distance: %.2f
                  node frame: %s
                  node frame parent: %s
                  sensor frame: %s,
                  node to world:
                  %s,
                  sensor to world:
                  %s
                  """).formatted(getClass().getSimpleName(),
                                 currentDistance,
                                 updatedCurrentDistance,
                                 getNodeFrame().getName(),
                                 getNodeFrame().getParent().getName(),
                                 sensorFrame.getName(),
                                 getNodeFrame().getTransformToWorldFrame(),
                                 sensorFrame.getTransformToWorldFrame()));
         }

         currentDistance = updatedCurrentDistance;
         //      if (currentDistance <= getDistanceToDisableTracking() && getTrackingInitialParent())
         //      {
         //         LogTools.warn("{}: Disabling tracking initial parent", getName());
         //         setTrackInitialParent(false, modificationQueue);
         //      }
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
      if (!MathTools.epsilonEquals(currentDistance, this.currentDistance, 0.1))
      {
//         LogTools.info(("""
//                           Type: %s
//                           Old distance: %.2f
//                           New distance: %.2f
//                        """).formatted(getClass().getSimpleName(),
//                                       this.currentDistance,
//                                       currentDistance));
      }

      this.currentDistance = currentDistance;
   }

   public double getCurrentDistance()
   {
      return currentDistance;
   }
}
