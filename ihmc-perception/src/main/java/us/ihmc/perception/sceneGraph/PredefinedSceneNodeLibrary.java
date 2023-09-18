package us.ihmc.perception.sceneGraph;

import gnu.trove.map.hash.TIntDoubleHashMap;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.rigidBodies.RigidBodySceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameSupplier;

import java.util.*;

/**
 * Use to specify which scene nodes a robot is looking for.
 *
 * We can put more specific stuff in here for now, like door specific
 * and even dynamic nodes.
 *
 * This is a super high level class, it includes ROS 2 topics,
 * heuristic stuff for certain nodes, stuff like that.
 *
 * This class exists so we can support multiple robots detecting the same
 * nodes, but also to serve the need we have to define things
 * by hand and construct custom heuristics.
 */
public class PredefinedSceneNodeLibrary
{
   private final ArUcoDetectableNode pushDoorPanel;
   private final ArUcoDetectableNode pullDoorPanel;
   private final ArUcoDetectableNode pushDoorLeverHandle;
   private final ArUcoDetectableNode pullDoorLeverHandle;
   private final StaticRelativeSceneNode pushDoorFrame;
   private final StaticRelativeSceneNode pullDoorFrame;
   private final ArUcoDetectableNode box;
   private final ArUcoDetectableNode canOfSoup;
   private final ArUcoDetectableNode debris;

   private final List<DetectableSceneNode> detectableSceneNodes = new ArrayList<>();
   private final List<ArUcoDetectableNode> arUcoDetectableNodes = new ArrayList<>();
   private final List<StaticRelativeSceneNode> staticArUcoRelativeDetectableNodes = new ArrayList<>();
   private final TIntDoubleHashMap arUcoMarkerIDsToSizes = new TIntDoubleHashMap();
   private final List<ReferenceFrameSupplier> referenceFrameSuppliers = new ArrayList<>();
   private final FramePose3D arUcoMarkerPose = new FramePose3D();

   public static PredefinedSceneNodeLibrary defaultObjects()
   {
      return new PredefinedSceneNodeLibrary();
   }

   private PredefinedSceneNodeLibrary()
   {
      // Add door stuff
      pushDoorPanel = DoorSceneNodeDefinitions.createPushDoorPanel();
      pullDoorPanel = DoorSceneNodeDefinitions.createPullDoorPanel();
      pushDoorLeverHandle = DoorSceneNodeDefinitions.createPushDoorLeverHandle();
      pullDoorLeverHandle = DoorSceneNodeDefinitions.createPullDoorLeverHandle();
      registerArUcoDetectableSceneNode(pushDoorPanel);
      registerArUcoDetectableSceneNode(pullDoorPanel);
      registerArUcoDetectableSceneNode(pushDoorLeverHandle);
      registerArUcoDetectableSceneNode(pullDoorLeverHandle);

      // The frames stay in place after being seen
      pushDoorFrame = DoorSceneNodeDefinitions.createPushDoorFrame(pushDoorPanel);
      pullDoorFrame = DoorSceneNodeDefinitions.createPullDoorFrame(pullDoorPanel);
      registerStaticArUcoRelativeDetectableSceneNode(pushDoorFrame);
      registerStaticArUcoRelativeDetectableSceneNode(pullDoorFrame);

      box = RigidBodySceneObjectDefinitions.createBox();
      canOfSoup = RigidBodySceneObjectDefinitions.createCanOfSoup();
      debris = RigidBodySceneObjectDefinitions.createDebris();
      registerArUcoDetectableSceneNode(box);
      registerArUcoDetectableSceneNode(canOfSoup);
      registerArUcoDetectableSceneNode(debris);

      // TODO: Add non-ArUco cup -- detected by neural net

   }

   public void registerArUcoDetectableSceneNode(ArUcoDetectableNode arUcoDetectableNode)
   {
      registerDetectableSceneNode(arUcoDetectableNode);
      arUcoDetectableNodes.add(arUcoDetectableNode);
      arUcoMarkerIDsToSizes.put(arUcoDetectableNode.getMarkerID(), arUcoDetectableNode.getMarkerSize());
   }

   public void registerStaticArUcoRelativeDetectableSceneNode(StaticRelativeSceneNode staticRelativeSceneNode)
   {
      registerDetectableSceneNode(staticRelativeSceneNode);
      staticArUcoRelativeDetectableNodes.add(staticRelativeSceneNode);
   }

   public void registerDetectableSceneNode(DetectableSceneNode detectableSceneNode)
   {
      detectableSceneNodes.add(detectableSceneNode);
      referenceFrameSuppliers.add(detectableSceneNode::getNodeFrame);
   }

   public void update(ReferenceFrame sensorFrame)
   {
      for (StaticRelativeSceneNode staticRelativeNode : staticArUcoRelativeDetectableNodes)
      {
         // We assume that the parent node is always an ArUco node, which is a weak assummption,
         // but the whole static relative thing is also weak.
         if (staticRelativeNode.getParentNode() instanceof ArUcoDetectableNode parentArUcoNode)
         {
            if (parentArUcoNode.getCurrentlyDetected())
            {
               arUcoMarkerPose.setToZero(parentArUcoNode.getMarkerFrame());
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
   }

   public List<DetectableSceneNode> getDetectableSceneNodes()
   {
      return detectableSceneNodes;
   }

   public List<ArUcoDetectableNode> getArUcoDetectableNodes()
   {
      return arUcoDetectableNodes;
   }

   public TIntDoubleHashMap getArUcoMarkerIDsToSizes()
   {
      return arUcoMarkerIDsToSizes;
   }

   public List<ReferenceFrameSupplier> getReferenceFrameSuppliers()
   {
      return referenceFrameSuppliers;
   }
}
