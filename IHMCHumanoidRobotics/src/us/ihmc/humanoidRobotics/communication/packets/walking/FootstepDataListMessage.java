package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message commands the controller to execute a list of footsteps. See FootstepDataMessage for information about defining a footstep."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/footstep_list")
public class FootstepDataListMessage extends Packet<FootstepDataListMessage> implements TransformableDataObject<FootstepDataListMessage>, Iterable<FootstepDataMessage>, VisualizablePacket
{
   @RosExportedField(documentation = "Defines the list of footstep to perform.")
   public ArrayList<FootstepDataMessage> footstepDataList = new ArrayList<FootstepDataMessage>();

   @RosExportedField(documentation = "When OVERRIDE is chosen:"
         + "\n - All previously sent footstep lists will be overriden, regardless of their execution mode"
         + "\n - A footstep can be overridden up to end of transfer. Once the swing foot leaves the ground it cannot be overridden"
         + "\n When QUEUE is chosen:"
         + "\n This footstep list will be queued with previously sent footstep lists, regardless of their execution mode"
         + "\n - The trajectory time and swing time of all queued footsteps will be overwritten with this message's values.")
   public ExecutionMode executionMode = ExecutionMode.OVERRIDE;

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
   public double defaultTransferDuration = 0.0;

   @RosExportedField(documentation = "Specifies the time used to return to a stable standing stance after the execution of the"
         + "\nfootstep list is finished. If the value is negative the defaultTransferDuration will be used.")
   public double finalTransferDuration = -1.0;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootstepDataListMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
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
      this.executionMode = executionMode;
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
      this.executionMode = ExecutionMode.OVERRIDE;
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


   public boolean epsilonEquals(FootstepDataListMessage otherList, double epsilon)
   {
      for (int i = 0; i < size(); i++)
      {
         if (!otherList.get(i).epsilonEquals(get(i), epsilon))
         {
            return false;
         }
      }

      if (!MathTools.epsilonEquals(this.defaultSwingDuration, otherList.defaultSwingDuration, epsilon))
      {
         return false;
      }

      if (!MathTools.epsilonEquals(this.defaultTransferDuration, otherList.defaultTransferDuration, epsilon))
      {
         return false;
      }

      if (this.executionMode != otherList.executionMode)
      {
         return false;
      }

      if (this.executionTiming != otherList.executionTiming)
      {
         return false;
      }

      if (!MathTools.epsilonEquals(this.finalTransferDuration, otherList.finalTransferDuration, epsilon))
      {
         return false;
      }

      return true;
   }

   public String toString()
   {
      String startingFootstep = "";
      int numberOfSteps = this.size();
      if (numberOfSteps > 0)
      {
         startingFootstep = this.get(0).getLocation().toString();
         Quaternion quat4d = this.get(0).getOrientation();

         FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), quat4d);
         startingFootstep = startingFootstep + ", ypr= " + Arrays.toString(frameOrientation.getYawPitchRoll());
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
               + "\tExecution Mode: " + this.executionMode + "\n"
               + "\tExecution Timing: " + this.executionTiming + "\n"
               + "\tTransfer Duration: " + this.defaultTransferDuration + "\n"
               + "\tSwing Duration: " + this.defaultSwingDuration + "\n"
               + "\tSize: " + this.size() + " Footsteps");
      }
   }

   public Iterator<FootstepDataMessage> iterator()
   {
      return footstepDataList.iterator();
   }

   public FootstepDataListMessage transform(RigidBodyTransform transform)
   {
      FootstepDataListMessage ret = new FootstepDataListMessage(defaultSwingDuration, defaultTransferDuration, finalTransferDuration);

      for (FootstepDataMessage footstepData : footstepDataList)
      {
         FootstepDataMessage transformedFootstepData = footstepData.transform(transform);
         ret.add(transformedFootstepData);
      }

      return ret;
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
      this.executionMode = executionMode;
   }

   public void setExecutionTiming(ExecutionTiming executionTiming)
   {
      this.executionTiming = executionTiming;
   }

   public ExecutionTiming getExecutionTiming()
   {
      return executionTiming;
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
      this.executionMode = RandomNumbers.nextEnum(random, ExecutionMode.class);
      this.executionTiming = RandomNumbers.nextEnum(random, ExecutionTiming.class);
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataListMessage(this);
   }
}
