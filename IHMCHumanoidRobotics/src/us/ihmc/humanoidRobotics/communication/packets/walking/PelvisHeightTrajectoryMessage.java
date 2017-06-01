package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.AbstractEuclideanTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@RosMessagePacket(documentation =
      "This mesage commands the controller to move the pelvis to a new height in world while going through the specified trajectory points."
      + " Sending this command will not affect the pelvis horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead."
      + " A third order polynomial is used to interpolate between trajectory points."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/pelvis_height_trajectory")
public class PelvisHeightTrajectoryMessage extends AbstractEuclideanTrajectoryMessage<PelvisHeightTrajectoryMessage> implements VisualizablePacket, TransformableDataObject<PelvisHeightTrajectoryMessage>
{
   /** Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics **/
   public boolean enableUserPelvisControl = false;
   
   /** 
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking} will keep the height manager in user mode while walking. If this is false the height
    * manager will switch to controller mode when walking
    **/
   public boolean enableUserPelvisControlDuringWalking = false;
   
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisHeightTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
      linearSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix.setAxisSelection(false, false, true);
   }

   public PelvisHeightTrajectoryMessage(Random random)
   {
      super(random);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
      linearSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix.setAxisSelection(false, false, true);
   }

   /**
    * Clone contructor.
    * @param pelvisHeightTrajectoryMessage message to clone.
    */
   public PelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      super(pelvisHeightTrajectoryMessage);
      setUniqueId(pelvisHeightTrajectoryMessage.getUniqueId());
      setDestination(pelvisHeightTrajectoryMessage.getDestination());

      executionMode = pelvisHeightTrajectoryMessage.executionMode;
      previousMessageId = pelvisHeightTrajectoryMessage.previousMessageId;
      enableUserPelvisControl = pelvisHeightTrajectoryMessage.enableUserPelvisControl;
      enableUserPelvisControlDuringWalking = pelvisHeightTrajectoryMessage.isEnableUserPelvisControlDuringWalking();
   }

   /**
    * Use this constructor to go straight to the given end point.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in data frame
    */
   public PelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight, ReferenceFrame trajectoryReferenceFrame, ReferenceFrame dataReferenceFrame)
   {
      super(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight),trajectoryReferenceFrame.getNameBasedHashCode());
      frameInformation.setDataReferenceFrame(dataReferenceFrame);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
      
      linearSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix.setAxisSelection(false, false, true);
   }

   /**
    * Use this constructor to go straight to the given end point.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in world frame.
    */
   public PelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      super(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight), ReferenceFrame.getWorldFrame());
      frameInformation.setDataReferenceFrame(ReferenceFrame.getWorldFrame());
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
      
      linearSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix.setAxisSelection(false, false, true);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, double, double)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public PelvisHeightTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
      
      linearSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix.setAxisSelection(false, false, true);
   }
   
   public boolean isEnableUserPelvisControl()
   {
      return enableUserPelvisControl;
   }

   public void setEnableUserPelvisControl(boolean enableUserPelvisControl)
   {
      this.enableUserPelvisControl = enableUserPelvisControl;
   }

   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   @Override
   public PelvisHeightTrajectoryMessage transform(RigidBodyTransform transform)
   {
      PelvisHeightTrajectoryMessage transformedMessage = new PelvisHeightTrajectoryMessage(this);
      for (int trajectoryPointIndex = 0; trajectoryPointIndex < getNumberOfTrajectoryPoints(); trajectoryPointIndex++)
      {
         EuclideanTrajectoryPointMessage trajectoryPoint = transformedMessage.getTrajectoryPoint(trajectoryPointIndex);
         trajectoryPoint.transform(transform);
      }
      return transformedMessage;
   }
   
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validatePelvisHeightTrajectoryMessage(this);
   }
   
   @Override
   public boolean epsilonEquals(PelvisHeightTrajectoryMessage other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      if (taskspaceTrajectoryPoints != null)
         return "Pelvis height trajectory: number of trajectory points = " + getNumberOfTrajectoryPoints();
      else
         return "Pelvis height trajectory: no trajectory points   .";
   }
}
