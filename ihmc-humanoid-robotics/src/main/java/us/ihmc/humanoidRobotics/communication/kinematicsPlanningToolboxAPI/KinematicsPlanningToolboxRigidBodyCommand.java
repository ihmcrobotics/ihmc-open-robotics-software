package us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI;

import java.util.Map;

import controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.WeightMatrix3DMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsPlanningToolboxRigidBodyCommand implements Command<KinematicsPlanningToolboxRigidBodyCommand, KinematicsPlanningToolboxRigidBodyMessage>,
      KinematicsPlanningToolboxAPI<KinematicsPlanningToolboxRigidBodyMessage>
{
   private long sequenceId;
   /** This is the unique hash code of the end-effector to be solved for. */
   private int endEffectorHashCode;
   /** This is the end-effector to be solved for. */
   private RigidBodyBasics endEffector;

   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();
   private final RecyclingArrayList<Pose3D> waypoints = new RecyclingArrayList<>(Pose3D.class);
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();

   private final FramePose3D controlFramePose = new FramePose3D();
   private TDoubleArrayList allowablePositionDisplacement = new TDoubleArrayList();
   private TDoubleArrayList allowableOrientationDisplacement = new TDoubleArrayList();

   @Override
   public void set(KinematicsPlanningToolboxRigidBodyCommand other)
   {
      clear();

      sequenceId = other.sequenceId;

      endEffectorHashCode = other.endEffectorHashCode;
      endEffector = other.endEffector;

      for (int i = 0; i < other.waypoints.size(); i++)
      {
         waypoints.add().set(other.waypoints.get(i));
         waypointTimes.add(other.waypointTimes.get(i));
      }
      selectionMatrix.set(other.selectionMatrix);
      weightMatrix.set(other.weightMatrix);

      controlFramePose.setIncludingFrame(other.controlFramePose);

      for (int i = 0; i < other.waypoints.size(); i++)
      {
         allowablePositionDisplacement.add(other.allowablePositionDisplacement.get(i));
         allowableOrientationDisplacement.add(other.allowableOrientationDisplacement.get(i));
      }
   }

   public void set(KinematicsPlanningToolboxRigidBodyMessage message, Map<Integer, RigidBodyBasics> rigidBodyHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();
      sequenceId = message.getSequenceId();
      endEffectorHashCode = message.getEndEffectorHashCode();
      if (rigidBodyHashMap == null)
         endEffector = null;
      else
         endEffector = rigidBodyHashMap.get(endEffectorHashCode);

      for (int i = 0; i < message.getKeyFramePoses().size(); i++)
      {
         waypoints.add().set(message.getKeyFramePoses().get(i));
         waypointTimes.add(message.getKeyFrameTimes().get(i));
      }

      selectionMatrix.clearSelectionFrame();
      SelectionMatrix3DMessage angularSelection = message.getAngularSelectionMatrix();
      SelectionMatrix3DMessage linearSelection = message.getLinearSelectionMatrix();
      selectionMatrix.setAngularAxisSelection(angularSelection.getXSelected(), angularSelection.getYSelected(), angularSelection.getZSelected());
      selectionMatrix.setLinearAxisSelection(linearSelection.getXSelected(), linearSelection.getYSelected(), linearSelection.getZSelected());

      weightMatrix.clear();
      WeightMatrix3DMessage angularWeight = message.getAngularWeightMatrix();
      WeightMatrix3DMessage linearWeight = message.getLinearWeightMatrix();
      weightMatrix.setAngularWeights(angularWeight.getXWeight(), angularWeight.getYWeight(), angularWeight.getZWeight());
      weightMatrix.setLinearWeights(linearWeight.getXWeight(), linearWeight.getYWeight(), linearWeight.getZWeight());

      if (referenceFrameResolver != null)
      {
         ReferenceFrame angularSelectionFrame = referenceFrameResolver.getReferenceFrame(angularSelection.getSelectionFrameId());
         ReferenceFrame linearSelectionFrame = referenceFrameResolver.getReferenceFrame(linearSelection.getSelectionFrameId());
         selectionMatrix.setSelectionFrames(angularSelectionFrame, linearSelectionFrame);
         ReferenceFrame angularWeightFrame = referenceFrameResolver.getReferenceFrame(angularWeight.getWeightFrameId());
         ReferenceFrame linearWeightFrame = referenceFrameResolver.getReferenceFrame(linearWeight.getWeightFrameId());
         weightMatrix.setWeightFrames(angularWeightFrame, linearWeightFrame);
      }

      ReferenceFrame referenceFrame = endEffector == null ? null : endEffector.getBodyFixedFrame();
      controlFramePose.setIncludingFrame(referenceFrame, message.getControlFramePositionInEndEffector(), message.getControlFrameOrientationInEndEffector());

      for (int i = 0; i < message.getAllowablePositionDisplacement().size(); i++)
      {
         allowablePositionDisplacement.add(message.getAllowablePositionDisplacement().get(i));
         allowableOrientationDisplacement.add(message.getAllowableOrientationDisplacement().get(i));
      }
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      endEffectorHashCode = 0;
      endEffector = null;
      waypoints.clear();
      waypointTimes.clear();
      selectionMatrix.resetSelection();
      weightMatrix.clear();

      controlFramePose.setToNaN(ReferenceFrame.getWorldFrame());

      allowablePositionDisplacement.clear();
      allowableOrientationDisplacement.clear();
   }

   @Override
   public void setFromMessage(KinematicsPlanningToolboxRigidBodyMessage message)
   {
      set(message, null, null);
   }

   @Override
   public Class<KinematicsPlanningToolboxRigidBodyMessage> getMessageClass()
   {
      return KinematicsPlanningToolboxRigidBodyMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return endEffector != null && waypoints.size() > 0 && waypoints.size() == waypointTimes.size()
            && waypointTimes.size() == allowablePositionDisplacement.size() && allowablePositionDisplacement.size() == allowableOrientationDisplacement.size();
   }

   public RigidBodyBasics getEndEffector()
   {
      return endEffector;
   }

   public int getNumberOfWayPoints()
   {
      return waypoints.size();
   }

   public TDoubleArrayList getWayPointTimes()
   {
      return waypointTimes;
   }

   public double getWayPointTime(int i)
   {
      return waypointTimes.get(i);
   }

   public Pose3D getWayPoint(int i)
   {
      return waypoints.get(i);
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   public FramePose3D getControlFramePose()
   {
      return controlFramePose;
   }

   public double getAllowablePositionDisplacement(int i)
   {
      return allowablePositionDisplacement.get(i);
   }

   public double getAllowableOrientationDisplacement(int i)
   {
      return allowableOrientationDisplacement.get(i);
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
