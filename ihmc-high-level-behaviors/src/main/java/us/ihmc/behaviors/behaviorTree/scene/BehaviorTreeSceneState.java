package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

/**
 * Instantiated in both RDX UI and on the robot.
 *
 * Synchronized as a CRDT.
 */
public class BehaviorTreeSceneState
{
   protected final ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();

   public BehaviorTreeSceneState()
   {

   }

   public ReferenceFrameLibrary getReferenceFrameLibrary()
   {
      return null;
   }
}
