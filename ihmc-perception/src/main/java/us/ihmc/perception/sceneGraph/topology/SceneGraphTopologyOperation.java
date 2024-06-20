package us.ihmc.perception.sceneGraph.topology;

/**
 * Describes and has everything needed to perform a modification of
 * the scene tree structure (topology). This is so they can be queued up
 * and performed with consistency guarantees.
 */
@FunctionalInterface
public interface SceneGraphTopologyOperation
{
   void performOperation();
}
