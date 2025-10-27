package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Instantiated in both RDX UI and on the robot.
 *
 * Synchronized as a CRDT.
 */
public class BehaviorTreeSceneState
{

   private final ArrayList<ReferenceFrame> robotFrames = new ArrayList<>();
   private final Map<String, ReferenceFrame> robotFrameMap = new HashMap<>();

   private final List<BehaviorTreeSceneObject> objects = new ArrayList<>();

   public BehaviorTreeSceneState(ROS2SyncedRobotModel syncedRobot)
   {
      addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
      addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      for (RobotSide side : RobotSide.values)
         if (syncedRobot.getRobotModel().getRobotVersion().hasArm(side))
            addAll(Collections.singleton(syncedRobot.getReferenceFrames().getHandZUpFrame(side)));
   }

   private void addAll(Collection<ReferenceFrame> referenceFrames)
   {
      robotFrames.addAll(referenceFrames);
      referenceFrames.forEach(referenceFrame -> robotFrameMap.put(referenceFrame.getName(), referenceFrame));
   }

   public boolean containsFrame(String referenceFrameName)
   {
      for (ReferenceFrame frame : robotFrames)
      {
         if (referenceFrameName.equals(frame.getName()))
            return true;
      }

      return false;
   }

   @Nullable
   public ReferenceFrame findFrameByName(String referenceFrameName)
   {
      // Check map first, then dynamic collections
      ReferenceFrame referenceFrame = robotFrameMap.get(referenceFrameName);
      boolean frameFound = referenceFrame != null;


      return frameFound ? referenceFrame : null;
   }

   public void getAllFrameNames(Consumer<String> frameNameConsumer)
   {
      for (ReferenceFrame frame : robotFrames)
      {
         frameNameConsumer.accept(frame.getName());
      }
   }


   public BehaviorTreeSceneObject getObject(String objectName)
   {
      return null;
   }

   public List<BehaviorTreeSceneObject> getObjects()
   {
      return objects;
   }
}
