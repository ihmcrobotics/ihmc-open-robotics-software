package us.ihmc.robotiq.simulatedHand;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.packetConsumers.FingerStateProvider;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.dataobjects.HandJointName;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotiq.model.RobotiqHandModel;
import us.ihmc.utilities.humanoidRobot.partNames.FingerName;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;

public class SimulatedRobotiqHandsController implements MultiThreadedRobotControlElement
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DoubleYoVariable handControllerTime = new DoubleYoVariable("handControllerTime", registry);
   private final LongYoVariable lastEstimatorStartTime = new LongYoVariable("nextExecutionTime", registry);
   private final BooleanYoVariable sendFingerJointGains = new BooleanYoVariable("sendFingerJointGains", registry);

   private final DoubleYoVariable kpFingerJoints = new DoubleYoVariable("kpFingerJoints", registry);
   private final DoubleYoVariable kdFingerJoints = new DoubleYoVariable("kdFingerJoints", registry);
   private final DoubleYoVariable fingerTrajectoryTime = new DoubleYoVariable("FingerTrajectoryTime", registry);

   private final long controlDTInNS;
   private final long estimatorDTInNS;

   private final ThreadDataSynchronizer threadDataSynchronizer;

   private final SideDependentList<FingerStateProvider> fingerStateProviders = new SideDependentList<>();

   private final SideDependentList<List<OneDegreeOfFreedomJoint>> allFingerJoints = new SideDependentList<>();

   private final RobotiqHandModel handModel = new RobotiqHandModel();

   private final SideDependentList<IndividualRobotiqHandController> individualHandControllers = new SideDependentList<>();

   private final SimulatedRobotiqHandJointAngleProducer jointAngleProducer;

   private final SideDependentList<Boolean> hasRobotiqHand = new SideDependentList<Boolean>(false, false);

   public SimulatedRobotiqHandsController(SDFRobot simulatedRobot, DRCRobotModel robotModel, ThreadDataSynchronizer threadDataSynchronizer,
         GlobalDataProducer globalDataProducer)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.controlDTInNS = TimeTools.secondsToNanoSeconds(robotModel.getControllerDT());
      this.estimatorDTInNS = TimeTools.secondsToNanoSeconds(robotModel.getEstimatorDT());
      sendFingerJointGains.set(true);

      jointAngleProducer = new SimulatedRobotiqHandJointAngleProducer(globalDataProducer, simulatedRobot);

      kpFingerJoints.set(100.0);
      kdFingerJoints.set(10.0);
      fingerTrajectoryTime.set(0.5);

      for (RobotSide robotSide : RobotSide.values)
      {
         allFingerJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());

         for (HandJointName jointEnum : handModel.getHandJointNames())
         {
            OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointEnum.getJointName(robotSide));
            if (fingerJoint != null)
               hasRobotiqHand.put(robotSide, true);
            allFingerJoints.get(robotSide).add(fingerJoint);
         }

         if (hasRobotiqHand.get(robotSide))
         {
            FingerStateProvider fingerStateProvider = new FingerStateProvider(robotSide);
            fingerStateProviders.put(robotSide, fingerStateProvider);
            if (globalDataProducer != null)
               globalDataProducer.getObjectCommunicator().attachListener(FingerStatePacket.class, fingerStateProvider);

            IndividualRobotiqHandController individualHandController = new IndividualRobotiqHandController(robotSide, handControllerTime, fingerTrajectoryTime,
                  simulatedRobot, registry);
            individualHandControllers.put(robotSide, individualHandController);
         }
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void read(long currentClockTime)
   {
      long timestamp = threadDataSynchronizer.getTimestamp();
      handControllerTime.set(TimeTools.nanoSecondstoSeconds(timestamp));
      jointAngleProducer.sendHandJointAnglesPacket();
   }

   @Override
   public void run()
   {
      checkForNewFingerStateRequested();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasRobotiqHand.get(robotSide))
            individualHandControllers.get(robotSide).computeDesiredJointAngles();
      }
   }

   private void checkForNewFingerStateRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!hasRobotiqHand.get(robotSide))
            continue;

         FingerStateProvider fingerStateProvider = fingerStateProviders.get(robotSide);
         if (fingerStateProvider.isNewFingerStateAvailable())
         {
            IndividualRobotiqHandController individualRobotiqHandController = individualHandControllers.get(robotSide);

            FingerState fingerState = fingerStateProvider.pullPacket().getFingerState();
            switch (fingerState)
            {
               case OPEN:
                  individualRobotiqHandController.open();
                  break;

               case OPEN_INDEX:
                  individualRobotiqHandController.open(FingerName.INDEX);
                  break;

               case OPEN_MIDDLE:
                  individualRobotiqHandController.open(FingerName.MIDDLE);
                  break;

               case OPEN_THUMB:
                  individualRobotiqHandController.open(FingerName.THUMB);
                  break;

               case CLOSE:
                  individualRobotiqHandController.close();
                  break;

               case CLOSE_FINGERS:
                  individualRobotiqHandController.close(FingerName.INDEX);
                  individualRobotiqHandController.close(FingerName.MIDDLE);
                  break;

               case CLOSE_THUMB:
                  individualRobotiqHandController.close(FingerName.THUMB);
                  break;

               case RESET:
                  individualRobotiqHandController.reset();
                  break;

               case HOOK:
                  individualRobotiqHandController.hook();
                  break;

               case CRUSH:
                  individualRobotiqHandController.crush();
                  break;

               case CRUSH_INDEX:
                  individualRobotiqHandController.crush(FingerName.INDEX);
                  break;

               case CRUSH_MIDDLE:
                  individualRobotiqHandController.crush(FingerName.MIDDLE);
                  break;

               case CRUSH_THUMB:
                  individualRobotiqHandController.crush(FingerName.THUMB);
                  break;

               default:
                  break;
            }
         }
      }
   }

   @Override
   public void write(long timestamp)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasRobotiqHand.get(robotSide))
         {
            List<OneDegreeOfFreedomJoint> oneSideFingerJoints = allFingerJoints.get(robotSide);
            for (int i = 0; i < oneSideFingerJoints.size(); i++)
            {
               OneDegreeOfFreedomJoint joint = oneSideFingerJoints.get(i);
               joint.setKp(kpFingerJoints.getDoubleValue());
               joint.setKd(kdFingerJoints.getDoubleValue());
            }
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasRobotiqHand.get(robotSide))
            individualHandControllers.get(robotSide).writeDesiredJointAngles();
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return null;
   }

   @Override
   public long nextWakeupTime()
   {
      if (lastEstimatorStartTime.getLongValue() == Long.MIN_VALUE)
      {
         return Long.MIN_VALUE;
      }
      else
      {
         return lastEstimatorStartTime.getLongValue() + controlDTInNS + estimatorDTInNS;
      }
   }

}
