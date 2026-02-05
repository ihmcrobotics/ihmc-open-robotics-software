package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
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
import java.util.function.LongSupplier;

/**
 * Instantiated in both RDX UI and on the robot.
 *
 * Synchronized as a CRDT.
 */
public abstract class BehaviorTreeSceneState
{
   protected final CRDTInfo crdtInfo;
   protected final LongSupplier idSupplier;
   protected final ROS2SyncedRobotModel syncedRobot;

   private final List<ReferenceFrame> robotFrames = new ArrayList<>();
   private final Map<String, ReferenceFrame> robotFrameMap = new HashMap<>();

   protected final LatestTimestampModifiable objectsModifiable;
   protected final List<BehaviorTreeSceneObjectState> objects = new ArrayList<>();

   public BehaviorTreeSceneState(CRDTInfo crdtInfo, LongSupplier idSupplier, ROS2SyncedRobotModel syncedRobot)
   {
      this.crdtInfo = crdtInfo;
      this.idSupplier = idSupplier;
      this.syncedRobot = syncedRobot;

      objectsModifiable = new LatestTimestampModifiable(crdtInfo);
      objectsModifiable.setModifierName("Scene objects");

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
      boolean contains = robotFrameMap.containsKey(referenceFrameName);

      if (!contains)
         for (BehaviorTreeSceneObjectState object : objects)
            if (referenceFrameName.startsWith(object.getName())) // TODO Affordance frame
               return true;

      return contains;
   }

   @Nullable
   public ReferenceFrame findFrameByName(String referenceFrameName)
   {
      // Check map first, then dynamic collections
      ReferenceFrame referenceFrame = robotFrameMap.get(referenceFrameName);

      if (referenceFrame == null)
         for (BehaviorTreeSceneObjectState object : objects)
            if (referenceFrameName.startsWith(object.getName())) // TODO Affordance frame
               return object.getReferenceFrame();

      return referenceFrame;
   }

   public void getAllFrameNames(Consumer<String> frameNameConsumer)
   {
      for (ReferenceFrame frame : robotFrames)
      {
         frameNameConsumer.accept(frame.getName());
      }

      for (BehaviorTreeSceneObjectState object : objects)
      {
         frameNameConsumer.accept(object.getName());
      }
   }

   public void toMessage(BehaviorTreeSceneStateMessage message)
   {
      objectsModifiable.toMessage(message.getLatestModificationToList());

      message.getObjects().clear();
      for (BehaviorTreeSceneObjectState object : objects)
         object.toMessage(message.getObjects().add());
   }

   public void fromMessage(BehaviorTreeSceneStateMessage message)
   {
      objectsModifiable.fromMessage(message.getLatestModificationToList());

      if (objectsModifiable.isModificationIncoming())
      {
         List<BehaviorTreeSceneObjectState> priorLocalObjects = new ArrayList<>(objects);
         objects.clear();

         for (BehaviorTreeSceneObjectStateMessage objectMessage : message.getObjects())
         {
            BehaviorTreeSceneObjectState localMatch = null;
            for (BehaviorTreeSceneObjectState priorLocalObject : priorLocalObjects)
               if (objectMessage.getId() == priorLocalObject.getID())
                  localMatch = priorLocalObject;

            if (localMatch == null) // Replicate
            {
               objects.add(buildObject(objectMessage));
            }
            else // Keep existing object
            {
               objects.add(localMatch);
               priorLocalObjects.remove(localMatch);
            }
         }

         for (BehaviorTreeSceneObjectState priorLocalObject : priorLocalObjects) // Destroy leftovers
            priorLocalObject.destroy();
      }

      // Update incoming nodes with the same IDs
      // TODO: Fix to not be O(n^2)
      for (BehaviorTreeSceneObjectStateMessage objectMessage : message.getObjects())
         for (BehaviorTreeSceneObjectState object : objects)
            if (objectMessage.getId() == object.getID())
               object.fromMessage(objectMessage);
   }

   protected abstract BehaviorTreeSceneObjectState buildObject(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition);

   public BehaviorTreeSceneObjectState createObject(BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      return buildObject(idSupplier.getAsLong(), crdtInfo, definition);
   }

   protected BehaviorTreeSceneObjectState buildObject(BehaviorTreeSceneObjectStateMessage message)
   {
      return buildObject(message.getId(), crdtInfo, message.getDefinition());
   }

   public BehaviorTreeSceneObjectState getObject(String objectName)
   {
      return null;
   }

   public BehaviorTreeSceneObjectState getObject(BehaviorTreeSceneObjectType objectType)
   {
      for (BehaviorTreeSceneObjectState object : objects)
      {
         if (object.getObjectType() == objectType)
         {
            return object;
         }
      }

      return null;
   }

   public void addObject(BehaviorTreeSceneObjectState object)
   {
      objects.add(object);
      objectsModifiable.modify();
   }

   public void removeAllObjects()
   {
      objects.forEach(BehaviorTreeSceneObjectState::destroy);
      objects.clear();
      objectsModifiable.modify();
   }

   public void removeObject(BehaviorTreeSceneObjectState object)
   {
      object.destroy();
      objects.remove(object);
      objectsModifiable.modify();
   }

   public List<BehaviorTreeSceneObjectState> getObjects()
   {
      return objects;
   }
}
