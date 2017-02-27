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

   @RosExportedField(documentation = "swingTime is the time spent in single-support when stepping"
    + "\n - Queueing does not support varying swing times. If execution mode is QUEUE, all queued footsteps' swingTime will be overwritten"
    + "\n - The value can be overwritten by specifying a swing time in the FootstepDataMessage for each individual step.")
   public double defaultSwingTime = 0.0;
   @RosExportedField(documentation = "transferTime is the time spent in double-support between steps"
   + "\n - Queueing does not support varying transfer times. If execution mode is QUEUE, all queued footsteps' transferTime will be overwritten"
   + "\n - The value can be overwritten by specifying a transfer time in the FootstepDataMessage for each individual step.")
   public double defaultTransferTime = 0.0;

   @RosExportedField(documentation = "Specifies the transfer time to go to standing after the execution of the footstep list is finished."
   + "\nIf the value is negative the defaultTransfer time will be used.")
   public double finalTransferTime = -1.0;

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
    * @param defaultSwingTime
    * @param defaultTransferTime
    * @param executionMode
    */
   public FootstepDataListMessage(ArrayList<FootstepDataMessage> footstepDataList, double defaultSwingTime, double defaultTransferTime, ExecutionMode executionMode)
   {
      this(footstepDataList, defaultSwingTime, defaultTransferTime, defaultTransferTime, executionMode);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param footstepDataList
    * @param defaultSwingTime
    * @param defaultTransferTime
    * @param finalTransferTime
    * @param executionMode
    */
   public FootstepDataListMessage(ArrayList<FootstepDataMessage> footstepDataList, double defaultSwingTime, double defaultTransferTime, double finalTransferTime,
         ExecutionMode executionMode)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      if(footstepDataList != null)
      {
         this.footstepDataList = footstepDataList;
      }
      this.defaultSwingTime = defaultSwingTime;
      this.defaultTransferTime = defaultTransferTime;
      this.finalTransferTime = finalTransferTime;
      this.executionMode = executionMode;
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. Set execution mode to OVERRIDE
    * @param defaultSwingTime
    * @param defaultTransferTime
    */
   public FootstepDataListMessage(double defaultSwingTime, double defaultTransferTime)
   {
      this(defaultSwingTime, defaultTransferTime, defaultTransferTime);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. Set execution mode to OVERRIDE
    * @param defaultSwingTime
    * @param defaultTransferTime
    * @param finalTransferTime
    */
   public FootstepDataListMessage(double defaultSwingTime, double defaultTransferTime, double finalTransferTime)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.defaultSwingTime = defaultSwingTime;
      this.defaultTransferTime = defaultTransferTime;
      this.finalTransferTime = finalTransferTime;
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

      if (!MathTools.epsilonEquals(this.defaultSwingTime, otherList.defaultSwingTime, epsilon))
      {
         return false;
      }

      if (!MathTools.epsilonEquals(this.defaultTransferTime, otherList.defaultTransferTime, epsilon))
      {
         return false;
      }

      if (this.executionMode != otherList.executionMode)
      {
         return false;
      }

      if (!MathTools.epsilonEquals(this.finalTransferTime, otherList.finalTransferTime, epsilon))
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
         return ("Starting Footstep: " + startingFootstep + "\n\tExecution Mode: " + this.executionMode + "\n\tTransfer Time: " + defaultTransferTime + "\n\tSwing Time: " + defaultSwingTime + "\n\tSize: " + this.size() + " Footsteps");
      }
   }

   public Iterator<FootstepDataMessage> iterator()
   {
      return footstepDataList.iterator();
   }

   public FootstepDataListMessage transform(RigidBodyTransform transform)
   {
      FootstepDataListMessage ret = new FootstepDataListMessage(defaultSwingTime, defaultTransferTime, finalTransferTime);

      for (FootstepDataMessage footstepData : footstepDataList)
      {
         FootstepDataMessage transformedFootstepData = footstepData.transform(transform);
         ret.add(transformedFootstepData);
      }

      return ret;
   }

   public void setDefaultSwingTime(double defaultSwingTime)
   {
      this.defaultSwingTime = defaultSwingTime;
   }

   public void setDefaultTransferTime(double defaultTransferTime)
   {
      this.defaultTransferTime = defaultTransferTime;
   }

   public void setFinalTransferTime(double finalTransferTime)
   {
      this.finalTransferTime = finalTransferTime;
   }

   public void setExecutionMode(ExecutionMode executionMode)
   {
      this.executionMode = executionMode;
   }

   public FootstepDataListMessage(Random random)
   {
      setUniqueId(1L);
      int footstepListSize = random.nextInt(20);
      for (int i = 0; i < footstepListSize; i++)
      {
         footstepDataList.add(new FootstepDataMessage(random));
      }

      this.defaultSwingTime = RandomNumbers.nextDoubleWithEdgeCases(random, 0.1);
      this.defaultTransferTime = RandomNumbers.nextDoubleWithEdgeCases(random, 0.1);
      this.finalTransferTime = RandomNumbers.nextDoubleWithEdgeCases(random, 0.1);
      this.executionMode = RandomNumbers.nextEnum(random, ExecutionMode.class);
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataListMessage(this);
   }
}
