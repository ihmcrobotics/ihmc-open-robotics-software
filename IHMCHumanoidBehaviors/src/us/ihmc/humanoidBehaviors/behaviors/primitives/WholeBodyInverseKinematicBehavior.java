package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIKPacketCreator;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;

public class WholeBodyInverseKinematicBehavior extends BehaviorInterface
{
   private static final double trajectoryDuration = 2.0;
   
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private final BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet" + behaviorName, registry);

   private final WholeBodyIKPacketCreator wholeBodyNetworkModule;
   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final SDFFullRobotModel actualFullRobotModel;
   private final SDFFullRobotModel desiredFullRobotModel;
   private final ArrayList<Packet> packetsToSend = new ArrayList<Packet>();

   public WholeBodyInverseKinematicBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge,
         WholeBodyControllerParameters wholeBodyControllerParameters, SDFFullRobotModel actualFullRobotModel)
   {
      super(outgoingCommunicationBridge);
      wholeBodyNetworkModule = new WholeBodyIKPacketCreator(wholeBodyControllerParameters);
      wholeBodyIKSolver = wholeBodyControllerParameters.createWholeBodyIkSolver();

      this.actualFullRobotModel = actualFullRobotModel;
      this.desiredFullRobotModel = wholeBodyControllerParameters.createFullRobotModel();
   }

   public void setInputs(RobotSide robotSide, FramePose endEffectorPose) throws Exception
   {
      wholeBodyIKSolver.setHandTarget(actualFullRobotModel, robotSide, endEffectorPose);
      
      wholeBodyIKSolver.compute(actualFullRobotModel, desiredFullRobotModel);
      hasInputBeenSet.set(true);
   }

   private void sendSolutionToController(double trajectoryDuration)
   {
      packetsToSend.clear();
      wholeBodyNetworkModule.createPackets(desiredFullRobotModel, trajectoryDuration, packetsToSend);
      for (int i = 0; i < packetsToSend.size(); i++)
      {
         sendPacketToController(packetsToSend.get(i));
      }
   }
   
   
   @Override
   public void doControl()
   {

      //TODO check all the status
      
      if (!packetHasBeenSent.getBooleanValue())
      {
         sendSolutionToController(trajectoryDuration);
         //TODO send packets to controller
         packetHasBeenSent.set(true);
      }
  
      
   }

   @Override
   public void initialize()
   {
      packetHasBeenSent.set(false);
      hasInputBeenSet.set(false);

      wholeBodyIKSolver.setPreferedJointPose("r_leg_kny", 1.5);//- 0.81

      wholeBodyIKSolver.setVerbose(false);
      wholeBodyIKSolver.getHierarchicalSolver().collisionAvoidance.setEnabled(false);

   }

   @Override
   public void finalize()
   {
      packetHasBeenSent.set(false);
      hasInputBeenSet.set(false);

      isPaused.set(false);
      isStopped.set(false);
   }

   @Override
   public void stop()
   {
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return false;
   }
}
