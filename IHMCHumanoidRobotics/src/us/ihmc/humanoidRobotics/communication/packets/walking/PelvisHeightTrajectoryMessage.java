package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.Abstract1DTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;

@RosMessagePacket(documentation =
      "This mesage commands the controller to move the pelvis to a new height in world while going through the specified trajectory points."
      + " Sending this command will not affect the pelvis horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead."
      + " A third order polynomial is used to interpolate between trajectory points."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/pelvis_height_trajectory")
public class PelvisHeightTrajectoryMessage extends Abstract1DTrajectoryMessage<PelvisHeightTrajectoryMessage> implements VisualizablePacket, TransformableDataObject<PelvisHeightTrajectoryMessage>
{
   @RosExportedField(documentation = "When OVERRIDE is chosen:"
         + "\n - The time of the first trajectory point can be zero, in which case the controller will start directly at the first trajectory point."
         + " Otherwise the controller will prepend a first trajectory point at the current desired position."
         + "\n When QUEUE is chosen:"
         + "\n - The message must carry the ID of the message it should be queued to."
         + "\n - The very first message of a list of queued messages has to be an OVERRIDE message."
         + "\n - The trajectory point times are relative to the the last trajectory point time of the previous message."
         + "\n - The controller will queue the joint trajectory messages as a per joint basis."
         + " The first trajectory point has to be greater than zero.")
   public ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   @RosExportedField(documentation = "Only needed when using QUEUE mode, it refers to the message Id to which this message should be queued to."
         + " It is used by the controller to ensure that no message has been lost on the way."
         + " If a message appears to be missing (previousMessageId different from the last message ID received by the controller), the motion is aborted."
         + " If previousMessageId == 0, the controller will not check for the ID of the last received message.")
   public long previousMessageId = INVALID_MESSAGE_ID;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisHeightTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
   }

   public PelvisHeightTrajectoryMessage(Random random)
   {
      super(random);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
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
   }

   /**
    * Use this constructor to go straight to the given end point.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in world frame.
    */
   public PelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      super(trajectoryTime, desiredHeight);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
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
   }

   /**
    * Set how the controller should consume this message:
    * <li> {@link ExecutionMode#OVERRIDE}: this message will override any previous message, including canceling any active execution of a message.
    * <li> {@link ExecutionMode#QUEUE}: this message is queued and will be executed once all the previous messages are done.
    * @param executionMode
    * @param previousMessageId when queuing, one needs to provide the ID of the message this message should be queued to.
    */
   public void setExecutionMode(ExecutionMode executionMode, long previousMessageId)
   {
      this.executionMode = executionMode;
      this.previousMessageId = previousMessageId;
   }

   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   public long getPreviousMessageId()
   {
      return previousMessageId;
   }

   @Override
   public boolean epsilonEquals(PelvisHeightTrajectoryMessage other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      if (trajectoryPoints != null)
         return "Pelvis height 1D trajectory: number of 1D trajectory points = " + getNumberOfTrajectoryPoints();
      else
         return "Pelvis height 1D trajectory: no 1D trajectory point.";
   }

   @Override
   public PelvisHeightTrajectoryMessage transform(RigidBodyTransform transform)
   {
      PelvisHeightTrajectoryMessage transformedMessage = new PelvisHeightTrajectoryMessage(this);
      Vector3D translation = new Vector3D();
      transform.getTranslation(translation);
      for (int trajectoryPointIndex = 0; trajectoryPointIndex < getNumberOfTrajectoryPoints(); trajectoryPointIndex++)
      {
         TrajectoryPoint1DMessage trajectoryPoint = transformedMessage.getTrajectoryPoint(trajectoryPointIndex);
         trajectoryPoint.setPosition(trajectoryPoint.getPosition() + translation.getZ());
      }
      return transformedMessage;
   }

   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validatePelvisHeightTrajectoryMessage(this);
   }
}
