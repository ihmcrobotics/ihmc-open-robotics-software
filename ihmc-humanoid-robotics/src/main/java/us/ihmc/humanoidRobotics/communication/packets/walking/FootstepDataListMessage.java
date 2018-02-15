package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message commands the controller to execute a list of footsteps. See FootstepDataMessage for information about defining a footstep."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/footstep_list")
public class FootstepDataListMessage extends Packet<FootstepDataListMessage>
{
   @RosExportedField(documentation = "Defines the list of footstep to perform.")
   public ArrayList<FootstepDataMessage> footstepDataList = new ArrayList<FootstepDataMessage>();

   @RosExportedField(documentation = "When CONTROL_DURATIONS is chosen:"
         + "\n The controller will try to achieve the swingDuration and the transferDuration specified in the message. If a"
         + "\n footstep touches down early, the next step will not be affected by this and the whole trajectory might finish"
         + "\n earlier then expected."
         + "\nWhen CONTROL_ABSOLUTE_TIMINGS is chosen:"
         + "\n The controller will compute the expected times for swing start and touchdown and attempt to start a footstep"
         + "\n at that time. If a footstep touches down early, the following transfer will be extended to make up for this"
         + "\n time difference and the footstep plan will finish at the expected time.")
   public ExecutionTiming executionTiming = ExecutionTiming.CONTROL_DURATIONS;

   @RosExportedField(documentation = "The swingDuration is the time a foot is not in ground contact during a step."
         + "\nEach step in a list of footsteps might have a different swing duration. The value specified here is a default"
         + "\nvalue, used if a footstep in this list was created without a swingDuration.")
   public double defaultSwingDuration = 0.0;
   @RosExportedField(documentation = "The transferDuration is the time spent with the feet in ground contact before a step."
         + "\nEach step in a list of footsteps might have a different transfer duration. The value specified here is a default"
         + "\nvalue, used if a footstep in this list was created without a transferDuration.")
   public double defaultTransferDuration = -1.0;

   @RosExportedField(documentation = "Specifies the time used to return to a stable standing stance after the execution of the"
         + "\nfootstep list is finished. If the value is negative the defaultTransferDuration will be used.")
   public double finalTransferDuration = -1.0;

   /** If{@code false} the controller adjust each footstep height to be at the support sole height. */
   public boolean trustHeightOfFootsteps = true;
   @RosExportedField(documentation = "Contains information on whether the robot can automatically adjust its footsteps to retain balance.")
   public boolean areFootstepsAdjustable = true;

   /** If {@code true} the controller will adjust upcoming footsteps with the location error of previous steps. */
   public boolean offsetFootstepsWithExecutionError = false;

   @RosExportedField(documentation = "Properties for queueing footstep lists.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootstepDataListMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public FootstepDataListMessage(FootstepDataListMessage other)
   {
      for(FootstepDataMessage otherFootstep : other.footstepDataList)
         footstepDataList.add(new FootstepDataMessage(otherFootstep));
      executionTiming = other.executionTiming;
      defaultSwingDuration = other.defaultSwingDuration;
      defaultTransferDuration = other.defaultTransferDuration;
      finalTransferDuration = other.finalTransferDuration;
      trustHeightOfFootsteps = other.trustHeightOfFootsteps;
      areFootstepsAdjustable = other.areFootstepsAdjustable;
      offsetFootstepsWithExecutionError = other.offsetFootstepsWithExecutionError;
      if (other.queueingProperties != null)
         queueingProperties.set(other.queueingProperties);
   }

   public FootstepDataListMessage(ArrayList<FootstepDataMessage> footstepDataList, double finalTransferDuration)
   {
      this(footstepDataList, 0.0, 0.0, finalTransferDuration, ExecutionMode.OVERRIDE);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param executionMode
    */
   public FootstepDataListMessage(ArrayList<FootstepDataMessage> footstepDataList, double defaultSwingDuration, double defaultTransferDuration, ExecutionMode executionMode)
   {
      this(footstepDataList, defaultSwingDuration, defaultTransferDuration, defaultTransferDuration, executionMode);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param finalTransferDuration
    * @param executionMode
    */
   public FootstepDataListMessage(ArrayList<FootstepDataMessage> footstepDataList, double defaultSwingDuration, double defaultTransferDuration, double finalTransferDuration,
         ExecutionMode executionMode)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      if(footstepDataList != null)
      {
         this.footstepDataList = footstepDataList;
      }
      this.defaultSwingDuration = defaultSwingDuration;
      this.defaultTransferDuration = defaultTransferDuration;
      this.finalTransferDuration = finalTransferDuration;
      queueingProperties.setExecutionMode(executionMode, Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. Set execution mode to OVERRIDE
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    */
   public FootstepDataListMessage(double defaultSwingDuration, double defaultTransferDuration)
   {
      this(defaultSwingDuration, defaultTransferDuration, defaultTransferDuration);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. Set execution mode to OVERRIDE
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param finalTransferDuration
    */
   public FootstepDataListMessage(double defaultSwingDuration, double defaultTransferDuration, double finalTransferDuration)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.defaultSwingDuration = defaultSwingDuration;
      this.defaultTransferDuration = defaultTransferDuration;
      this.finalTransferDuration = finalTransferDuration;
      queueingProperties.setExecutionMode(ExecutionMode.OVERRIDE, Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public ArrayList<FootstepDataMessage> getDataList()
   {
      return footstepDataList;
   }

   public void add(FootstepDataMessage footstepData)
   {
      this.footstepDataList.add(footstepData);
   }

   public void clear()
   {
      this.footstepDataList.clear();
   }

   public FootstepDataMessage get(int i)
   {
      return this.footstepDataList.get(i);
   }

   public int size()
   {
      return this.footstepDataList.size();
   }


   @Override
   public boolean epsilonEquals(FootstepDataListMessage otherList, double epsilon)
   {
      if (!queueingProperties.epsilonEquals(otherList.queueingProperties, epsilon))
         return false;

      for (int i = 0; i < size(); i++)
      {
         if (!otherList.get(i).epsilonEquals(get(i), epsilon))
         {
            return false;
         }
      }

      if (!MathTools.epsilonCompare(this.defaultSwingDuration, otherList.defaultSwingDuration, epsilon))
      {
         return false;
      }

      if (!MathTools.epsilonCompare(this.defaultTransferDuration, otherList.defaultTransferDuration, epsilon))
      {
         return false;
      }

      if (this.executionTiming != otherList.executionTiming)
      {
         return false;
      }

      if (!MathTools.epsilonCompare(this.finalTransferDuration, otherList.finalTransferDuration, epsilon))
      {
         return false;
      }

      if (this.offsetFootstepsWithExecutionError != otherList.offsetFootstepsWithExecutionError)
      {
         return false;
      }

      return true;
   }

   @Override
   public String toString()
   {
      String startingFootstep = "";
      int numberOfSteps = this.size();
      if (numberOfSteps > 0)
      {
         startingFootstep = this.get(0).getLocation().toString();
         Quaternion quat4d = this.get(0).getOrientation();

         FrameQuaternion frameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), quat4d);
         startingFootstep = startingFootstep + ", yaw= " + frameOrientation.getYaw() + ", pitch= " + frameOrientation.getPitch() + ", roll= " + frameOrientation.getRoll();
      }

      if (this.size() == 1)
      {
         RobotSide footSide = footstepDataList.get(0).getRobotSide();
         String side = "Right Step";
         if (footSide.getSideNameFirstLetter().equals("L"))
            side = "Left Step";

         return (side);
      }
      else
      {
         return ("Starting Footstep: " + startingFootstep + "\n"
               + "\tExecution Mode: " + queueingProperties.getExecutionMode().toString() + "\n"
               + "\tExecution Timing: " + this.executionTiming + "\n"
               + "\tTransfer Duration: " + this.defaultTransferDuration + "\n"
               + "\tSwing Duration: " + this.defaultSwingDuration + "\n"
               + "\tSize: " + this.size() + " Footsteps");
      }
   }

   public void setDefaultSwingDuration(double defaultSwingDuration)
   {
      this.defaultSwingDuration = defaultSwingDuration;
   }

   public void setDefaultTransferDuration(double defaultTransferDuration)
   {
      this.defaultTransferDuration = defaultTransferDuration;
   }

   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration = finalTransferDuration;
   }

   public void setExecutionMode(ExecutionMode executionMode)
   {
      queueingProperties.setExecutionMode(executionMode, VALID_MESSAGE_DEFAULT_ID);
   }

   public void setExecutionTiming(ExecutionTiming executionTiming)
   {
      this.executionTiming = executionTiming;
   }

   public ExecutionTiming getExecutionTiming()
   {
      return executionTiming;
   }

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return queueingProperties.getExecutionDelayTime();
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   public void setExecutionDelayTime(double delayTime)
   {
      queueingProperties.setExecutionDelayTime(delayTime);
   }

   public void setTrustHeightOfFootsteps(boolean trustHeight)
   {
      trustHeightOfFootsteps = trustHeight;
   }

   public void setAreFootstepsAdjustable(boolean areFootstepsAdjustable)
   {
      this.areFootstepsAdjustable = areFootstepsAdjustable;
   }

   public void setOffsetFootstepsWithExecutionError(boolean offsetFootstepsWithExecutionError)
   {
      this.offsetFootstepsWithExecutionError = offsetFootstepsWithExecutionError;
   }

   public boolean isOffsetFootstepsWithExecutionError()
   {
      return offsetFootstepsWithExecutionError;
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }

   public FootstepDataListMessage(Random random)
   {
      setUniqueId(1L);
      int footstepListSize = random.nextInt(20);
      for (int i = 0; i < footstepListSize; i++)
      {
         footstepDataList.add(new FootstepDataMessage(random));
      }

      this.defaultSwingDuration = RandomNumbers.nextDoubleWithEdgeCases(random, 0.1);
      this.defaultTransferDuration = RandomNumbers.nextDoubleWithEdgeCases(random, 0.1);
      this.finalTransferDuration = RandomNumbers.nextDoubleWithEdgeCases(random, 0.1);
      this.executionTiming = RandomNumbers.nextEnum(random, ExecutionTiming.class);
      queueingProperties = new QueueableMessage(random);
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataListMessage(this);
   }
}
