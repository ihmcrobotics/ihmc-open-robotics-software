package us.ihmc.perception.sceneGraph;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;

public class SceneGraphTest
{
   @Test
   public void testBasicOperations()
   {
      SceneGraph sceneGraph = new SceneGraph();

      Assertions.assertEquals(0, sceneGraph.getRootNode().getChildren().size());

      Assertions.assertEquals(1, sceneGraph.getNextID().getValue());
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

      Assertions.assertEquals(2, sceneGraph.getNextID().getValue());
      Assertions.assertEquals(2, sceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(2, sceneGraph.getNodeNameList().size());
      Assertions.assertEquals(2, sceneGraph.getNamesToNodesMap().size());
      Assertions.assertEquals(2, sceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(0, sceneGraph.getArUcoMarkerIDToNodeMap().size());



   }
}
