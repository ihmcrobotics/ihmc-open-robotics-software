package us.ihmc.valkyrie.fingers;

import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.SDFBaseRobot;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.packetConsumers.FingerStateProvider;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.tools.time.TimeTools;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class ValkyrieFingerController implements MultiThreadedRobotControlElement
{
   private final boolean DEBUG = true;
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final boolean isRunningOnRealRobot;
   private final SDFFullRobotModel fullRobotModel;
   
   private final DoubleYoVariable fingerControllerTime = new DoubleYoVariable("fingerControllerTime", registry);
   private final LongYoVariable lastEstimatorStartTime = new LongYoVariable("lastEstimatorStartTime", registry);
   private final BooleanYoVariable sendFingerJointGains = new BooleanYoVariable("sendFingerJointGains", registry);
   private final DoubleYoVariable fingerTrajectoryTime = new DoubleYoVariable("FingerTrajectoryTime", registry);
   
   private final SideDependentList<LinkedHashMap<ValkyrieSimulatedFingerJoint, DoubleYoVariable>> kpMap = new SideDependentList<>();
   private final SideDependentList<LinkedHashMap<ValkyrieSimulatedFingerJoint, DoubleYoVariable>> kdMap = new SideDependentList<>();
   
   private final long controlDTInNS;
   private final long estimatorDTInNS;

   private final ThreadDataSynchronizerInterface threadDataSynchronizer;

   private final SideDependentList<FingerStateProvider> fingerStateProviders = new SideDependentList<>();
   private final SideDependentList<ValkyrieFingerSetController> fingerSetControllers = new SideDependentList<>();

   private final SideDependentList<HandJointAngleCommunicator> jointAngleCommunicators = new SideDependentList<>();
   
   /**
    * @param robotModel
    * @param fullRobotModel null if running on real robot
    * @param threadDataSynchronizer
    * @param globalDataProducer
    * @param yoVariableRegistry 
    */
   public ValkyrieFingerController(DRCRobotModel robotModel, SDFBaseRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer, GlobalDataProducer globalDataProducer, YoVariableRegistry controllerRegistry)
   {
      this.isRunningOnRealRobot = robotModel.getStateEstimatorParameters().isRunningOnRealRobot();
      PrintTools.debug(DEBUG, "Running on real robot: " + isRunningOnRealRobot);
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.fullRobotModel = threadDataSynchronizer.getControllerFullRobotModel();
      this.controlDTInNS = TimeTools.secondsToNanoSeconds(robotModel.getControllerDT());
      this.estimatorDTInNS = TimeTools.secondsToNanoSeconds(robotModel.getEstimatorDT());
      sendFingerJointGains.set(true);
      fingerTrajectoryTime.set(0.5);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isRunningOnRealRobot)
         {
            kpMap.put(robotSide, new LinkedHashMap<ValkyrieSimulatedFingerJoint, DoubleYoVariable>());
            kdMap.put(robotSide, new LinkedHashMap<ValkyrieSimulatedFingerJoint, DoubleYoVariable>());
            
            for (ValkyrieSimulatedFingerJoint simulatedFingerJoint : ValkyrieSimulatedFingerJoint.values)
            {
               DoubleYoVariable kp = new DoubleYoVariable("kp" + robotSide.getCamelCaseNameForMiddleOfExpression() + simulatedFingerJoint.name(), registry);
               DoubleYoVariable kd = new DoubleYoVariable("kd" + robotSide.getCamelCaseNameForMiddleOfExpression() + simulatedFingerJoint.name(), registry);
               
               double kpkp = 2.0;
               double kdkd = 0.1;
               
               if (simulatedFingerJoint.name().endsWith("ThumbPitch1") || simulatedFingerJoint.name().endsWith("ThumbRoll"))
               {
                  kp.set(kpkp);
                  kd.set(kdkd);
               }
               else if (simulatedFingerJoint.name().endsWith("ThumbPitch2"))
               {
                  kp.set(kpkp / 2.0);
                  kd.set(kdkd / 2.0);
               }
               else if (simulatedFingerJoint.name().endsWith("ThumbPitch3"))
               {
                  kp.set(kpkp / 10.0);
                  kd.set(kdkd / 5.0);
               }
               else if (simulatedFingerJoint.name().endsWith("Pitch1"))
               {
                  kp.set(kpkp / 2.0);
                  kd.set(kdkd / 2.0);
               }
               else if (simulatedFingerJoint.name().endsWith("Pitch2"))
               {
                  kp.set(kpkp / 4.0);
                  kd.set(kdkd / 4.0);
               }
               else if (simulatedFingerJoint.name().endsWith("Pitch3"))
               {
                  kp.set(kpkp / 20.0);
                  kd.set(kdkd / 10.0);
               }
               
               kpMap.get(robotSide).put(simulatedFingerJoint, kp);
               kdMap.get(robotSide).put(simulatedFingerJoint, kd);
            }
         }
         
         fingerStateProviders.put(robotSide, new FingerStateProvider(robotSide));
         if (globalDataProducer != null)
            globalDataProducer.attachListener(FingerStatePacket.class, fingerStateProviders.get(robotSide));
         fingerSetControllers.put(robotSide, new ValkyrieFingerSetController(robotSide, fingerControllerTime, fingerTrajectoryTime, fullRobotModel, isRunningOnRealRobot, registry, controllerRegistry));
         
         jointAngleCommunicators.put(robotSide, new HandJointAngleCommunicator(robotSide, globalDataProducer));
      }
      
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isRunningOnRealRobot)
         {
            for (ValkyrieSimulatedFingerJoint simulatedFingerJoint : ValkyrieSimulatedFingerJoint.values)
            {               
               PinJoint relatedPinJoint = simulatedFingerJoint.getRelatedPinJoint(robotSide, simulatedRobot);
               RevoluteJoint relatedRevoluteJoint = (RevoluteJoint) simulatedFingerJoint.getRelatedRevoluteJoint(robotSide, fullRobotModel);
               
               relatedRevoluteJoint.setKp(kpMap.get(robotSide).get(simulatedFingerJoint).getDoubleValue());
               relatedRevoluteJoint.setKd(kdMap.get(robotSide).get(simulatedFingerJoint).getDoubleValue());
               
               double fullyExtensonPositionLimit = ValkyrieFingerJointLimits.getFullyExtensonPositionLimit(robotSide, simulatedFingerJoint.getRelatedRealFingerJoint());
               double fullyFlexedPositionLimit = ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, simulatedFingerJoint.getRelatedRealFingerJoint());
               
               if (fullyExtensonPositionLimit <= fullyFlexedPositionLimit)
                  relatedPinJoint.setLimitStops(fullyExtensonPositionLimit - 0.1, fullyFlexedPositionLimit + 0.1, 10.0, 2.5);
               else
                  relatedPinJoint.setLimitStops(fullyFlexedPositionLimit - 0.1, fullyExtensonPositionLimit + 0.1, 10.0, 2.5);
            }
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
      fingerControllerTime.set(TimeTools.nanoSecondstoSeconds(timestamp));
      
      for (RobotSide robotSide : RobotSide.values)
      {
         final double[] simulatedJointValues = new double[ValkyrieSimulatedFingerJoint.values.length];
         
         for (int i = 0; i < ValkyrieSimulatedFingerJoint.values.length; i++)
         {
            simulatedJointValues[i] = ValkyrieSimulatedFingerJoint.values[i].getRelatedRevoluteJoint(robotSide, fullRobotModel).getQ();
         }
         
         jointAngleCommunicators.get(robotSide).updateHandAngles(new HandSensorData()
         {
            @Override
            public boolean isConnected()
            {
               return true;
            }
            
            @Override
            public boolean isCalibrated()
            {
               return true;
            }
            
            @Override
            public double[][] getFingerJointAngles(RobotSide robotSide)
            {
               return new double[][] {simulatedJointValues, new double[] {}, new double[] {}};
            }
         });
         
         jointAngleCommunicators.get(robotSide).write();
      }
   }

   @Override
   public void run()
   {
      checkForNewFingerStateRequested();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         fingerSetControllers.get(robotSide).doControl();
      }
   }
   
   private void checkForNewFingerStateRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (fingerStateProviders.get(robotSide).isNewFingerStateAvailable())
         {
            FingerState fingerState = fingerStateProviders.get(robotSide).pullPacket().getFingerState();
            
            PrintTools.debug(DEBUG, this, "Recieved new FingerState Packet: " + fingerState);
            
            switch (fingerState)
            {
               case OPEN:
                  fingerSetControllers.get(robotSide).open();
                  break;

               case CLOSE:
                  fingerSetControllers.get(robotSide).close();
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
         if (!isRunningOnRealRobot)
         {
            for (ValkyrieSimulatedFingerJoint simulatedFingerJoint : ValkyrieSimulatedFingerJoint.values)
            {
               simulatedFingerJoint.getRelatedRevoluteJoint(robotSide, fullRobotModel).setKp(kpMap.get(robotSide).get(simulatedFingerJoint).getDoubleValue());
               simulatedFingerJoint.getRelatedRevoluteJoint(robotSide, fullRobotModel).setKd(kdMap.get(robotSide).get(simulatedFingerJoint).getDoubleValue());
            }
         }
         
         fingerSetControllers.get(robotSide).writeDesiredJointAngles();
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
