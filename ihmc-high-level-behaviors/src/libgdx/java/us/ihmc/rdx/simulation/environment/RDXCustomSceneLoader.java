package us.ihmc.rdx.simulation.environment;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.rdx.perception.sceneGraph.RDXPredefinedRigidBodySceneNode;
import us.ihmc.rdx.perception.sceneGraph.RDXSceneGraphUI;
import us.ihmc.rdx.perception.sceneGraph.builder.RDXPredefinedRigidBodySceneNodeBuilder;

public class RDXCustomSceneLoader
{
   public enum RDXDemoScene
   {
      EXPLOSIVE_BREACHING,
      MANIPULATION_2X4,
      NAVIGATION_BARRIER;
   }
   private final RDXSceneGraphUI sceneGraphUI;
   private final RDXPredefinedRigidBodySceneNodeBuilder predefinedRigidBodySceneNodeBuilder;

   public RDXCustomSceneLoader(RDXSceneGraphUI sceneGraphUI)
   {
      this.sceneGraphUI = sceneGraphUI;
      this.predefinedRigidBodySceneNodeBuilder = new RDXPredefinedRigidBodySceneNodeBuilder(sceneGraphUI.getSceneGraph());
   }
   public void loadCustomScene(RDXDemoScene demoScene)
   {
      switch (demoScene)
      {
         case EXPLOSIVE_BREACHING:
            // Addition of custom nodes in custom locations
            addNode("DoorLever", 1);
            setNodePose("DoorLever1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-87.56950), 0.0, 0.0),
                                                             new Point3D(1.70865, 2.41354, 0.88848)));

            addNode("DoorPanel", 1);
            setNodePose("DoorPanel1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(92.38699), 0.0, 0.0),
                                                             new Point3D(2.03492, 2.41911, 0.00)));

            addNode("Person", 1);
            setNodePose("Person1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(121.92967), 0.0, 0.0),
                                                          new Point3D(2.28347, -1.56887, 0.00)));

            addNode("Person", 2);
            setNodePose("Person2", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(121.92967), 0.0, 0.0),
                                                          new Point3D(5.49537, -3.56887, 0.00)));

            addNode("Charge", 1);
            setNodePose("Charge1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(165.47115), Math.toRadians(1.47664), Math.toRadians(-0.38263)),
                                                          new Point3D(1.66921, -1.70972, 1.00789)));

            addNode("Barrier", 1);
            setNodePose("Barrier1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(86.38002), 0.0, 0.0),
                                                          new Point3D(2.30070, -0.68487, 0.00000)));
            break;

         case MANIPULATION_2X4:
            addNode("DoorPanel", 1);
            setNodePose("DoorPanel1", new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0),
                                                             new Point3D(2.67600, 1.33025, 0.00)));

            addNode("PieceOfWood", 1);
            setNodePose("PieceOfWood1", new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0),
                                                             new Point3D(1.80786, -0.63325, 0.05815)));

            addNode("Table", 1);
            setNodePose("Table1", new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0),
                                                       new Point3D(1.95596, -0.87503, 0.0)));
            break;

         case NAVIGATION_BARRIER:
            // Addition of custom nodes in custom locations
            addNode("DoorLever", 1);
            setNodePose("DoorLever1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-87.56950), 0.0, 0.0),
                                                             new Point3D(1.70865, 2.41354, 0.88848)));

            addNode("DoorPanel", 1);
            setNodePose("DoorPanel1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(92.38699), 0.0, 0.0),
                                                             new Point3D(2.03492, 2.41911, 0.00)));

            addNode("Person", 1);
            setNodePose("Person1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(121.92967), 0.0, 0.0),
                                                          new Point3D(2.28347, -1.56887, 0.00)));

            addNode("Person", 2);
            setNodePose("Person2", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(121.92967), 0.0, 0.0),
                                                          new Point3D(5.49537, -3.56887, 0.00)));

            addNode("Person", 3);
            setNodePose("Person3", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(-173.81712), 0.0, 0.0),
                                                          new Point3D(2.28347, 0.59619, 0.00)));

            addNode("Barrier", 1);
            setNodePose("Barrier1", new RigidBodyTransform(new YawPitchRoll(Math.toRadians(86.38002), 0.0, 0.0),
                                                           new Point3D(2.30070, -0.68487, 0.00000)));
            break;

         default:
            throw new IllegalArgumentException("Unknown demo scene: " + demoScene);
      }
   }

   private void addNode(String nodeName, int id)
   {
      RDXPredefinedRigidBodySceneNode node = predefinedRigidBodySceneNodeBuilder.build(nodeName, nodeName + id);
      sceneGraphUI.getSceneGraph().modifyTree(modificationQueue ->
                                              {
                                                 modificationQueue.accept(new SceneGraphNodeAddition(node.getSceneNode(),
                                                                                                     predefinedRigidBodySceneNodeBuilder.getParent()));
                                                 sceneGraphUI.addUISceneNode(node);
                                              });
   }

   private void setNodePose(String nodeName, RigidBodyTransform  rigidBodyTransform)
   {
      sceneGraphUI.getSceneGraph()
                  .getNamesToNodesMap()
                  .get(nodeName)
                  .setNodeToParentFrameTransformAndUpdate(rigidBodyTransform);
   }
}
