package us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI;

import java.util.ArrayList;
import java.util.List;

import toolbox_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class KinematicsPlanningToolboxMessageFactory
{
   public static final double DEFAULT_POSITION_DISPLACEMENT = 0.01;
   public static final double DEFAULT_ORIENTATION_DISPLACEMENT = 0.01;

   public static void setDefaultAllowableDisplacement(KinematicsPlanningToolboxRigidBodyMessage message, int numberOfKeyFrames)
   {
      for (int i = 0; i < numberOfKeyFrames; i++)
      {
         message.getAllowablePositionDisplacement().add(DEFAULT_POSITION_DISPLACEMENT);
         message.getAllowableOrientationDisplacement().add(DEFAULT_ORIENTATION_DISPLACEMENT);
      }
   }

   public static KinematicsPlanningToolboxRigidBodyMessage holdRigidBodyCurrentPose(RigidBodyBasics rigidBody, TDoubleArrayList keyFrameTimes)
   {
      List<Pose3DReadOnly> currentPoses = new ArrayList<Pose3DReadOnly>();
      for (int i = 0; i < keyFrameTimes.size(); i++)
         currentPoses.add(new Pose3D(rigidBody.getBodyFixedFrame().getTransformToWorldFrame()));
      KinematicsPlanningToolboxRigidBodyMessage message = HumanoidMessageTools.createKinematicsPlanningToolboxRigidBodyMessage(rigidBody, keyFrameTimes,
                                                                                                                               currentPoses);

      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));

      return message;
   }
}
