package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.BehaviorTreeSceneObjectStateMessage;
import behavior_msgs.BehaviorTreeSceneStateMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import javax.annotation.Nullable;

/**
 * Instantiated in both RDX UI and on the robot.
 *
 * Synchronized as a CRDT.
 */
public abstract class BehaviorTreeSceneState
{
   protected final CRDTInfo crdtInfo;
   protected final LongSupplier idSupplier;
   protected ROS2SyncedRobotModel syncedRobot;

   private final List<Pair<String, ReferenceFrame>> robotFrames = new ArrayList<>();
   private final Map<String, ReferenceFrame> robotFrameMap = new HashMap<>();

   protected final LatestTimestampModifiable objectsModifiable;
   protected final CRDTBidirectionalFloat poseFilterAlpha;
   protected final CRDTBidirectionalFloat acceptanceConfidence;
   protected final CRDTBidirectionalFloat stabilityFrequency;
   protected final CRDTBidirectionalFloat historyDuration;
   protected final List<BehaviorTreeSceneObjectState> objects = new ArrayList<>();

   public BehaviorTreeSceneState(CRDTInfo crdtInfo, LongSupplier idSupplier, ROS2SyncedRobotModel syncedRobot)
   {
      this.crdtInfo = crdtInfo;
      this.idSupplier = idSupplier;
      this.syncedRobot = syncedRobot;

      objectsModifiable = new LatestTimestampModifiable(crdtInfo);
      objectsModifiable.setModifierName("Scene objects");
      poseFilterAlpha = new CRDTBidirectionalFloat(objectsModifiable, 0.5f);
      acceptanceConfidence = new CRDTBidirectionalFloat(objectsModifiable, 0.25f);
      stabilityFrequency = new CRDTBidirectionalFloat(objectsModifiable, 1.0f);
      historyDuration = new CRDTBidirectionalFloat(objectsModifiable, 2.0f);

      rebuildRobotFrames(syncedRobot);
   }

   private void add(String name, ReferenceFrame referenceFrame)
   {
      robotFrames.add(Pair.of(name, referenceFrame));
      robotFrameMap.put(name, referenceFrame);
   }

   public void setSyncedRobot(ROS2SyncedRobotModel syncedRobot)
   {
      if (this.syncedRobot == syncedRobot)
         return;

      this.syncedRobot = syncedRobot;

      rebuildRobotFrames(syncedRobot);
   }

   private void rebuildRobotFrames(ROS2SyncedRobotModel syncedRobot)
   {
      robotFrames.clear();
      robotFrameMap.clear();

      add("Chest", syncedRobot.getReferenceFrames().getChestFrame());
      add("Pelvis", syncedRobot.getReferenceFrames().getPelvisFrame());
      add("Walking", syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasArm(side))
            add(side.getPascalCaseName() + " Hand", syncedRobot.getReferenceFrames().getHandFrame(side));
         add(side.getPascalCaseName() + " Foot Sole", syncedRobot.getReferenceFrames().getSoleFrame(side));
      }
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
      for (Pair<String, ReferenceFrame> frame : robotFrames)
      {
         frameNameConsumer.accept(frame.getKey());
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

      message.setPoseFilterAlpha(poseFilterAlpha.toMessage());
      message.setAcceptanceConfidence(acceptanceConfidence.toMessage());
      message.setStabilityFrequency(stabilityFrequency.toMessage());
      message.setHistoryDuration(historyDuration.toMessage());
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

      poseFilterAlpha.fromMessage(message.getPoseFilterAlpha());
      acceptanceConfidence.fromMessage(message.getAcceptanceConfidence());
      stabilityFrequency.fromMessage(message.getStabilityFrequency());
      historyDuration.fromMessage(message.getHistoryDuration());
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

   public boolean pollHasStatus()
   {
      return true;
   }

   public float getPoseFilterAlpha()
   {
      return poseFilterAlpha.getValue();
   }

   public void setPoseFilterAlpha(float poseFilterAlpha)
   {
      this.poseFilterAlpha.setValue(poseFilterAlpha);
   }

   public float getAcceptanceConfidence()
   {
      return acceptanceConfidence.getValue();
   }

   public void setAcceptanceConfidence(float acceptanceConfidence)
   {
      this.acceptanceConfidence.setValue(acceptanceConfidence);
   }

   public float getStabilityFrequency()
   {
      return stabilityFrequency.getValue();
   }

   public void setStabilityFrequency(float stabilityFrequency)
   {
      this.stabilityFrequency.setValue(stabilityFrequency);
   }

   public float getHistoryDuration()
   {
      return historyDuration.getValue();
   }

   public void setHistoryDuration(float historyDuration)
   {
      this.historyDuration.setValue(historyDuration);
   }
}
