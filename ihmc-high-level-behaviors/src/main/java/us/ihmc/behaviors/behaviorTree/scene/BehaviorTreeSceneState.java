package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.ArrayList;
import java.util.List;

/**
 * Instantiated in both RDX UI and on the robot.
 *
 * Synchronized as a CRDT.
 */
public class BehaviorTreeSceneState
{
   private final List<BehaviorTreeSceneObject> objects = new ArrayList<>();
   protected final ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();

   public BehaviorTreeSceneState()
   {

   }

   public BehaviorTreeSceneObject getObject(String objectName)
   {
      return null;
   }

   public List<BehaviorTreeSceneObject> getObjects()
   {
      return objects;
   }

   public ReferenceFrameLibrary getReferenceFrameLibrary()
   {
      return referenceFrameLibrary;
   }
}
