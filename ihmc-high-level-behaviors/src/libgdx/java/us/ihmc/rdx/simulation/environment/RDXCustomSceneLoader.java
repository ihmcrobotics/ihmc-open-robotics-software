package us.ihmc.rdx.simulation.environment;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.rdx.perception.sceneGraph.RDXPredefinedRigidBodySceneNode;
import us.ihmc.rdx.perception.sceneGraph.RDXSceneGraphUI;
import us.ihmc.rdx.perception.sceneGraph.builder.RDXPredefinedRigidBodySceneNodeBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Map;

public class RDXCustomSceneLoader
{
   public enum RDXDemoScene
   {
      FLAT_GROUND,
      ROUGH_TERRAIN,
      EXPLOSIVE_BREACHING_A,
      ROOM_WITH_OBJECTS,
      MANIPULATION_2X4,
      NAVIGATION_BARRIER;
   }
   private final RDXSceneGraphUI sceneGraphUI;
   private final RDXPredefinedRigidBodySceneNodeBuilder predefinedRigidBodySceneNodeBuilder;
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2Helper ros2;

   public RDXCustomSceneLoader(RDXSceneGraphUI sceneGraphUI)
   {
      this.sceneGraphUI = sceneGraphUI;
      this.predefinedRigidBodySceneNodeBuilder = new RDXPredefinedRigidBodySceneNodeBuilder(sceneGraphUI.getSceneGraph());
   }

   public RDXCustomSceneLoader(RDXSceneGraphUI sceneGraphUI, ROS2Helper ros2, ROS2SyncedRobotModel syncedRobot)
   {
      this.sceneGraphUI = sceneGraphUI;
      this.predefinedRigidBodySceneNodeBuilder = new RDXPredefinedRigidBodySceneNodeBuilder(sceneGraphUI.getSceneGraph());
      this.syncedRobot = syncedRobot;
      this.ros2 = ros2;
   }

   public void loadCustomScene(RDXDemoScene demoScene)
   {
      switch (demoScene)
      {
         case ROOM_WITH_OBJECTS:
            // Addition of custom nodes in custom locations
            addNode("Shoe");
            setNodePose("Shoe1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-154.90819), 0.0, Math.toRadians(90.11775)),
                                                                  new Point3D(1.70407, -1.54288, 0.06497)));
            addNode("Shoe");
            setNodePose("Shoe2", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(178.84531), 0.0, Math.toRadians(93.10753)),
                                                        new Point3D(2.06348, -1.46410, 0.06497)));
            addNode("Couch");
            setNodePose("Couch1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-176.40176), 0.0, Math.toRadians(0.0)),
                                                        new Point3D(2.75043, -2.49085, 0.19621)));
            addNode("TrashCan");
            setNodePose("TrashCan1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-131.60989), 0.0, Math.toRadians(0.0)),
                                                         new Point3D(1.99680, 1.15527, 0.48898)));
            addNode("Charge");
            setNodePose("Charge1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-180.0), Math.toRadians(-88.09734), Math.toRadians(180.0)),
                                                            new Point3D(1.98124, 1.15255, 1.13201)));
            addNode("Table");
            setNodePose("Table1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(89.98926), Math.toRadians(0.0), Math.toRadians(0.0)),
                                                          new Point3D(2.90433, -0.00015, 0.45231)));
            addNode("Drill");
            setNodePose("Drill1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-94.64304), Math.toRadians(0.21260), Math.toRadians(-0.22175)),
                                                          new Point3D(2.72352, -0.58619, 1.01969)));
            break;
         case EXPLOSIVE_BREACHING_A:
            // Addition of custom nodes in custom locations
            addNode("DoorPullHandle");
            setNodePose("DoorPullHandle1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-92.89080), 0.0, 0.0),
                                                                  new Point3D(1.63098, 3.23158, 1.33200)));

            addNode("DoorPanel");
            setNodePose("DoorPanel1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(85.34664), 0.0, 0.0),
                                                             new Point3D(1.83915, 2.30509+0.97246, 0.00)));

            addNode("Person");
            setNodePose("Person1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(176.92345), 0.0, 0.0),
                                                          new Point3D(2.57299, 0.13116, 0.00)));

            addNode("Person");
            setNodePose("Person2", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(133.77302), 0.0, 0.0),
                                                          new Point3D(2.64078, -2.69888, 0.00)));

            addNode("Charge");
            setNodePose("Charge1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(83.11072), Math.toRadians(-0.18205), Math.toRadians(-1.50489)),
                                                          new Point3D(2.28759, -2.37839, 1.17361)));

            addNode("Barrier");
            setNodePose("Barrier1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(86.38002), 0.0, 0.0),
                                                           new Point3D(1.53626, 0.98736, 0.00000)));

            addNode("Barrier");
            setNodePose("Barrier2", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-92.06846), 0.0, 0.0),
                                                           new Point3D(1.75723, 0.96312, 0.79907)));

            addNode("Barrier");
            setNodePose("Barrier3", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-92.06846), 0.0, 0.0),
                                                           new Point3D(2.46589, 0.91928, 0.00000)));
            break;

         case MANIPULATION_2X4:
            addNode("DoorPanel");
            setNodePose("DoorPanel1", new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0),
                                                             new Point3D(2.67600, 1.33025, 0.00)));

            addNode("PieceOfWood");
            setNodePose("PieceOfWood1", new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0),
                                                               new Point3D(1.80786, -0.63325, 0.05815)));

            addNode("Table");
            setNodePose("Table1", new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0),
                                                         new Point3D(1.95596, -0.87503, 0.0)));
            break;

         case NAVIGATION_BARRIER:
            // Addition of custom nodes in custom locations
            addNode("DoorLever");
            setNodePose("DoorLever1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-87.56950), 0.0, 0.0),
                                                             new Point3D(1.70865, 2.41354, 0.88848)));

            addNode("DoorPanel");
            setNodePose("DoorPanel1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(92.38699), 0.0, 0.0),
                                                             new Point3D(2.03492, 2.41911, 0.00)));

            addNode("Person");
            setNodePose("Person1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(121.92967), 0.0, 0.0),
                                                          new Point3D(2.28347, -1.56887, 0.00)));

            addNode("Person");
            setNodePose("Person2", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(121.92967), 0.0, 0.0),
                                                          new Point3D(5.49537, -3.56887, 0.00)));

            addNode("Person");
            setNodePose("Person3", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-173.81712), 0.0, 0.0),
                                                          new Point3D(2.28347, 0.59619, 0.00)));

            addNode("Barrier");
            setNodePose("Barrier1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(86.38002), 0.0, 0.0),
                                                           new Point3D(2.30070, -0.68487, 0.00000)));
            break;

         default:
            break;
      }
   }

   private void addNode(String nodeName)
   {
      RDXPredefinedRigidBodySceneNode node = predefinedRigidBodySceneNodeBuilder.build(nodeName, nodeName);
      sceneGraphUI.getSceneGraph().modifyTree(modificationQueue ->
                                              {
                                                 modificationQueue.accept(new SceneGraphNodeAddition(node.getSceneNode(),
                                                                                                     predefinedRigidBodySceneNodeBuilder.getParent(),
                                                                                                     sceneGraphUI.getSceneGraph()));
                                                 sceneGraphUI.addUISceneNode(node);
                                              });
   }

   private void setNodePose(String nodeName, RigidBodyTransform  rigidBodyTransform)
   {
      if (sceneGraphUI.getSceneGraph().getNamesToNodesMap().get(nodeName) != null)
      {
         sceneGraphUI.getSceneGraph()
                     .getNamesToNodesMap()
                     .get(nodeName)
                     .setNodeToParentFrameTransformAndUpdate(rigidBodyTransform);
      }
   }

   public String getEnvironmentName(RDXDemoScene demoScene)
   {
      return switch (demoScene)
      {
         case EXPLOSIVE_BREACHING_A -> "BreachingDemoA.json";
         case ROOM_WITH_OBJECTS -> "RoomWithObjects.json";
         case ROUGH_TERRAIN -> "HarderTerrain.json";
         default -> "FlatGround.json";
      };
   }

   public void trackEnvironment(ArrayList<RDXEnvironmentObject> objects)
   {
      Map<String, SceneNode> sceneNodesMap = sceneGraphUI.getSceneGraph().getNamesToNodesMap();
      for (RDXEnvironmentObject object : objects)
      {
         int nodeIndex = object.getObjectIndex() + 1;
         SceneNode sceneNode = sceneNodesMap.get(object.getName() + nodeIndex);
         if (sceneNode != null)
         {
            try
            {
               object.setTransformToWorld(sceneNode.getNodeFrame().getTransformToWorldFrame());
            }
            catch (Exception e)
            {
               // Exception is caught and ignored. We still get some "is not a Rotation matrix" errors
               LogTools.warn(e);
            }
         }
      }
   }

   public void moveManipulatedObject()
   {
      ros2.subscribeViaCallback(AutonomyAPI.AI2R_STATUS, message ->
      {
         String objectGrasped = message.getObjectGraspedAsString();
         if (objectGrasped != null && !objectGrasped.isEmpty())
         {
            try
            {
               ReferenceFrame handFrame = syncedRobot.getFullRobotModel().getHandControlFrame(RobotSide.fromByte(message.getGraspSide()));
               FramePose3D objectTransform = new FramePose3D(handFrame, message.getTransformGraspedObjectHand());
               objectTransform.changeFrame(ReferenceFrame.getWorldFrame());
               setNodePose(objectGrasped, new RigidBodyTransform(objectTransform.getOrientation(), objectTransform.getTranslation()));
            }
            catch (Exception e)
            {
               // Exception is caught and ignored. We still get some "is not a Rotation matrix" errors
               LogTools.warn(e);
            }
         }
      });
   }
}