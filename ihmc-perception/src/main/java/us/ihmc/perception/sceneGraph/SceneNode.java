package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.tools.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * Represents a node on the Scene Knowledge Graph (to be defined).
 *
 * We give each node a name and a reference frame.
 */
public class SceneNode
{
   public static final Consumer<SceneNode> NOOP = sceneNode -> { };

   /** A linearly increasing ID */
   private final long id;
   private final String name;
   private final ModifiableReferenceFrame nodeFrame;
   private final List<SceneNode> children = new ArrayList<>();
   /**
    * Scene nodes are usually being synced at 20 Hz or faster, so 1/5 of a second
    * should allow enough time for changes to propagate.
    */
   public static final double OPERATOR_FREEZE_TIME = 1.0;
   /**
    * This timer is used in the case that an operator can "mark modified" this node's
    * data so it won't accept updates from other sources for a short period of time.
    * This is to allow the changes to propagate elsewhere.
    */
   private final Timer modifiedTimer = new Timer();

   public SceneNode(long id, String name)
   {
      this.id = id;
      this.name = name;
      this.nodeFrame = new ModifiableReferenceFrame(name, ReferenceFrame.getWorldFrame());
   }

   public void update()
   {
      update(NOOP);
   }

   public void update(Consumer<SceneNode> updateHeuristic)
   {
      for (SceneNode child : getChildren())
      {
         if (child.getNodeFrame().getParent() != getNodeFrame())
         {
            child.changeParentFrame(getNodeFrame());
         }

         updateHeuristic.accept(child);
         child.update(updateHeuristic);
      }
   }

   public long getID()
   {
      return id;
   }

   public String getName()
   {
      return name;
   }

   public ReferenceFrame getNodeFrame()
   {
      return nodeFrame.getReferenceFrame();
   }

   /**
    * Used to get and set the transform to the parent frame.
    * If you modify this transform, you must then call {@link ReferenceFrame#update()} on {@link #getNodeFrame()}.
    * @return the transform to the parent frame
    */
   public RigidBodyTransform getNodeToParentFrameTransform()
   {
      return nodeFrame.getTransformToParent();
   }

   protected void changeParentFrame(ReferenceFrame newParentFrame)
   {
      nodeFrame.changeParentFrame(newParentFrame);
   }

   protected void changeParentFrameWithoutMoving(ReferenceFrame newParentFrame)
   {
      nodeFrame.changeParentFrameWithoutMoving(newParentFrame);
   }

   public List<SceneNode> getChildren()
   {
      return children;
   }

   public void markModifiedByOperator()
   {
      modifiedTimer.reset();
   }

   public boolean operatorModifiedThisRecently()
   {
      return modifiedTimer.isRunning(OPERATOR_FREEZE_TIME);
   }
}
