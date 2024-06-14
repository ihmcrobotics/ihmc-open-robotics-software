package us.ihmc.perception.sceneGraph;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.robotics.EuclidCoreTestMissingTools;

public class SceneGraphTest
{
   private static final double EPSILON = 1.0e-15;

   @Test
   public void testBasicOperations()
   {
      SceneGraph sceneGraph = new SceneGraph();

      Assertions.assertEquals(0, sceneGraph.getRootNode().getChildren().size());

      Assertions.assertEquals(1, sceneGraph.getNextID().get());
      Assertions.assertEquals(1, sceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(1, sceneGraph.getNodeNameList().size());
      Assertions.assertEquals(1, sceneGraph.getNamesToNodesMap().size());
      Assertions.assertEquals(1, sceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(0, sceneGraph.getArUcoMarkerIDToNodeMap().size());

      sceneGraph.modifyTree(modificationQueue ->
      {
         modificationQueue.accept(new SceneGraphNodeAddition(new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child0"), sceneGraph.getRootNode()));
      });

      Assertions.assertEquals(1, sceneGraph.getRootNode().getChildren().size());

      Assertions.assertEquals(2, sceneGraph.getNextID().get());
      Assertions.assertEquals(2, sceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(2, sceneGraph.getNodeNameList().size());
      Assertions.assertEquals(2, sceneGraph.getNamesToNodesMap().size());
      Assertions.assertEquals(2, sceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(0, sceneGraph.getArUcoMarkerIDToNodeMap().size());

      SceneNode child1 = new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child1");
      sceneGraph.modifyTree(modificationQueue ->
      {
         modificationQueue.accept(new SceneGraphNodeAddition(child1, sceneGraph.getRootNode()));
         modificationQueue.accept(new SceneGraphNodeAddition(new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child1Child0"), child1));
      });

      Assertions.assertEquals(2, sceneGraph.getRootNode().getChildren().size());
      Assertions.assertEquals(1, child1.getChildren().size());

      Assertions.assertEquals(4, sceneGraph.getNextID().get());
      Assertions.assertEquals(4, sceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(4, sceneGraph.getNodeNameList().size());
      Assertions.assertEquals(4, sceneGraph.getNamesToNodesMap().size());
      Assertions.assertEquals(4, sceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(0, sceneGraph.getArUcoMarkerIDToNodeMap().size());
   }

   @Test
   public void testMovingNodes()
   {
      SceneGraph sceneGraph = new SceneGraph();
      SceneNode child0 = new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child0");
      SceneNode child1 = new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child1");
      SceneNode child1child0 = new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child1Child0");
      sceneGraph.modifyTree(modificationQueue ->
      {
         modificationQueue.accept(new SceneGraphNodeAddition(child0, sceneGraph.getRootNode()));
         modificationQueue.accept(new SceneGraphNodeAddition(child1, sceneGraph.getRootNode()));
         modificationQueue.accept(new SceneGraphNodeAddition(child1child0, child1));
      });

      child1child0.getModifiableNodeFrame().update(transformToParent -> transformToParent.getTranslation().set(5.0, 2.0, 1.0));

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(child1child0.getNodeFrame().getTransformToWorldFrame()));
      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString("""
                                                            1.000000000000000000 0.000000000000000000 0.000000000000000000 | 5.000000000000000000
                                                            0.000000000000000000 1.000000000000000000 0.000000000000000000 | 2.000000000000000000
                                                            0.000000000000000000 0.000000000000000000 1.000000000000000000 | 1.000000000000000000
                                                            0.000000000000000000 0.000000000000000000 0.000000000000000000 | 1.000000000000000000
                                                                  """), child1child0.getNodeFrame().getTransformToWorldFrame(), EPSILON);

      child1.getModifiableNodeFrame().update(transformToParent -> transformToParent.getTranslation().set(-2.0, 3.0, 0.5));

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(child1.getNodeFrame().getTransformToWorldFrame()));
      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            1.000000000000000000 0.000000000000000000 0.000000000000000000 | -2.000000000000000000
                                                            0.000000000000000000 1.000000000000000000 0.000000000000000000 | 3.000000000000000000
                                                            0.000000000000000000 0.000000000000000000 1.000000000000000000 | 0.500000000000000000
                                                            0.000000000000000000 0.000000000000000000 0.000000000000000000 | 1.000000000000000000
                                                                  """), child1.getNodeFrame().getTransformToWorldFrame(), EPSILON);
      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(child1child0.getNodeFrame().getTransformToWorldFrame()));
      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            1.000000000000000000 0.000000000000000000 0.000000000000000000 | 3.000000000000000000
                                                            0.000000000000000000 1.000000000000000000 0.000000000000000000 | 5.000000000000000000
                                                            0.000000000000000000 0.000000000000000000 1.000000000000000000 | 1.500000000000000000
                                                            0.000000000000000000 0.000000000000000000 0.000000000000000000 | 1.000000000000000000
                                                                  """), child1child0.getNodeFrame().getTransformToWorldFrame(), EPSILON);
   }
}
