package us.ihmc.perception.sceneGraph.modification;

/**
 * Describes and has everything needed to perform a modification of
 * the scene graph tree structure. This is so they can be queued up
 * and performed inside the scene graph with consistency guarantees.
 */
@FunctionalInterface
public interface SceneGraphTreeModification
{
   void performOperation();
}
