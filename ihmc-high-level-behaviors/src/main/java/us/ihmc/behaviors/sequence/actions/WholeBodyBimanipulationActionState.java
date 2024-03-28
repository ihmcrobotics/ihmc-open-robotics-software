package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class WholeBodyBimanipulationActionState extends ActionNodeState<WholeBodyBimanipulationActionDefinition>
{
   private final SideDependentList<CRDTDetachableReferenceFrame> handFrames = new SideDependentList<>();
   private final CRDTUnidirectionalDoubleArray jointAngles;
   private final CRDTUnidirectionalDouble solutionQuality;

   public WholeBodyBimanipulationActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new WholeBodyBimanipulationActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      for (RobotSide side : RobotSide.values)
      {
         CRDTDetachableReferenceFrame handFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary,
                                                          getDefinition().getCRDTParentFrameName(),
                                                          getDefinition().getHandToParentTransform(side));
         handFrames.put(side, handFrame);
      }

      jointAngles = new CRDTUnidirectionalDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, ArmJointAnglesActionDefinition.MAX_NUMBER_OF_JOINTS);
      solutionQuality = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
   }

   @Override
   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         handFrames.get(side).update();
      }
   }

   public void toMessage(WholeBodyBimanipulationActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      for (int i = 0; i < WholeBodyBimanipulationActionDefinition.MAX_NUMBER_OF_JOINTS; i++)
      {
         jointAngles.toMessage(message.getJointAngles());
      }
      message.setSolutionQuality(solutionQuality.toMessage());
   }

   public void fromMessage(WholeBodyBimanipulationActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      jointAngles.fromMessage(message.getJointAngles());
      solutionQuality.fromMessage(message.getSolutionQuality());
   }

   public CRDTDetachableReferenceFrame getHandFrame(RobotSide side)
   {
      return handFrames.get(side);
   }

   public double[] getJointAngles()
   {
      return jointAngles.getValue();
   }

   public void setJointAngles(double[] value)
   {
      for (int i = 0; i < ArmJointAnglesActionDefinition.MAX_NUMBER_OF_JOINTS; i++)
      {
         jointAngles.getValue()[i] = value[i];
      }
   }

   public double getSolutionQuality()
   {
      return solutionQuality.getValue();
   }

   public void setSolutionQuality(double solutionQuality)
   {
      this.solutionQuality.setValue(solutionQuality);
   }
}
