package us.ihmc.humanoidBehaviors.behaviors.debug;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class TestVariableSwingTimeBehavior extends AbstractBehavior
{
   private final HumanoidReferenceFrames referenceFrames;

   public TestVariableSwingTimeBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorEntered()
   {
      sendPacket(new TextToSpeechPacket("Starting to walk in 1s with varying swing times."));
      ThreadTools.sleep(1000);

      double finalTransferTime = 5.0;
      FootstepDataListMessage footsteps = new FootstepDataListMessage(1.2, 0.6, finalTransferTime);
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      RigidBodyTransform transformToWorld = referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame();
      PrintTools.info(transformToWorld.toString());

      for (int stepIndex = 0; stepIndex < 10; stepIndex++)
      {
         RobotSide side = stepIndex % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double y = side == RobotSide.LEFT ? 0.15 : -0.15;
         Point3d location = new Point3d(0.2 * (stepIndex + 1), y, 0.0);
         Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = new FootstepDataMessage(side, location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         double swingTime, transferTime;

         switch (stepIndex)
         {
         case 0:
            // start with very slow swings and transfers
            transferTime = 1.0; // initial transfer
            swingTime = 3.0;
            footstepData.setTimings(swingTime, transferTime);
            break;
         case 1:
            // do a default step
            break;
         case 2:
            // do a slow swing
            transferTime = 1.0;
            swingTime = 3.0;
            footstepData.setTimings(swingTime, transferTime);
            break;
         case 3:
            // do a default step
            break;
         case 4:
            // do a default step
            break;
         case 5:
            // do a fast swing and transfer
            transferTime = 0.2;
            swingTime = 0.6;
            footstepData.setTimings(swingTime, transferTime);
            break;
         case 6:
            // do a slow swing
            transferTime = 1.0;
            swingTime = 3.0;
            footstepData.setTimings(swingTime, transferTime);
            break;
         case 7:
            // do a fast swing and transfer
            transferTime = 0.2;
            swingTime = 0.6;
            footstepData.setTimings(swingTime, transferTime);
            break;
         case 8:
            // do a slow swing
            transferTime = 1.0;
            swingTime = 3.0;
            footstepData.setTimings(swingTime, transferTime);
            break;
         case 9:
            // do a slow transfer and a fast swing
            transferTime = 3.0;
            swingTime = 0.6;
            footstepData.setTimings(swingTime, transferTime);
            break;
         default:
            break;
         }

         footsteps.add(footstepData.transform(transformToWorld));
      }

      sendPacket(footsteps);
   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }
}
