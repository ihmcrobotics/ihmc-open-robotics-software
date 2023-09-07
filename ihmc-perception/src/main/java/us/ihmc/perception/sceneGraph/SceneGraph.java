package us.ihmc.perception.sceneGraph;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;

/**
 * A scene graph implementation that is really a scene tree. There
 * are no loops. This data structure is also a CRDT. It is synced between
 * the robot and operator UIs.
 *
 * The scene graph can be used to represent things a robot is looking for.
 */
public class SceneGraph
{
   public static final MutableInt NEXT_ID = new MutableInt();

   private final SceneNode rootNode = new SceneNode(NEXT_ID.getAndIncrement(), "SceneGraphRoot");

   private final FramePose3D arUcoMarkerPose = new FramePose3D();

   public SceneGraph()
   {
      // Here so that you can track the instantiations of this class with the IDE
   }

   public void update(ReferenceFrame sensorFrame)
   {
      rootNode.update(sceneNode ->
      {
         if (sceneNode instanceof StaticRelativeSceneNode staticRelativeNode)
         {
            // We assume that the parent node is always an ArUco node, which is a weak assummption,
            // but the whole static relative thing is also weak.
            if (staticRelativeNode.getParentNode() instanceof ArUcoMarkerNode parentArUcoNode)
            {
               if (parentArUcoNode.getCurrentlyDetected())
               {
                  arUcoMarkerPose.setToZero(parentArUcoNode.getNodeFrame());
                  arUcoMarkerPose.setFromReferenceFrame(sensorFrame);
                  double currentDistance = arUcoMarkerPose.getPosition().distanceFromOrigin();
                  staticRelativeNode.setCurrentDistance(currentDistance);
                  if (currentDistance <= staticRelativeNode.getDistanceToDisableTracking())
                  {
                     staticRelativeNode.setTrackDetectedPose(false);
                  }
               }
            }
         }
      });
   }

   public SceneNode getRootNode()
   {
      return rootNode;
   }
}
