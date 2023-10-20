package us.ihmc.perception.sceneGraph.modification;

import java.util.function.Consumer;

/**
 * This interface just exists to provide a better name to what this is,
 * which gets passed down from SceneGraph's modifyTree method and serves
 * to queue up tree modifications.
 */
public interface SceneGraphModificationQueue extends Consumer<SceneGraphTreeModification>
{

}
