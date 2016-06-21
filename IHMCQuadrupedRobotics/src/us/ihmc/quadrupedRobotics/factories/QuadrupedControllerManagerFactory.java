package us.ihmc.quadrupedRobotics.factories;

import java.io.IOException;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.quadrupedRobotics.communication.QuadrupedGlobalDataProducer;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerManager;
import us.ihmc.quadrupedRobotics.controller.forceDevelopment.QuadrupedForceDevelopmentControllerManager;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedStateEstimatorFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedControllerManagerFactory
{
   // PROVIDED
   private double controlDT;
   private double gravity;
   private DoubleYoVariable timestampYoVariable;
   private SDFFullQuadrupedRobotModel fullRobotModel;
   private YoVariableRegistry robotYoVariableRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private IHMCCommunicationKryoNetClassList kryoNetClassList;
   private QuadrupedPhysicalProperties physicalProperties;
   private QuadrupedReferenceFrames referenceFrames;
   private QuadrupedControlMode controlMode;
   
   // TO CONSTRUCT
   private PacketCommunicator packetCommunicator;
   private GlobalDataProducer globalDataProducer;
   private QuadrantDependentList<ContactablePlaneBody> contactableFeet;
   private QuadrantDependentList<FootSwitchInterface> footSwitches;
   private QuadrupedRuntimeEnvironment runtimeEnvironment;
   
   // CREATION
   
   private void createContactableFeet()
   {
      contactableFeet = QuadrupedStateEstimatorFactory.createFootContactableBodies(fullRobotModel, referenceFrames, physicalProperties);
   }
   
   private void createFootSwitches()
   {
      footSwitches = QuadrupedStateEstimatorFactory.createFootSwitches(contactableFeet, gravity, fullRobotModel, robotYoVariableRegistry);
   }
   
   private void createPacketCommunicator() throws IOException
   {
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, kryoNetClassList);
      packetCommunicator.connect();
   }
   
   private void createGlobalDataProducer()
   {
      globalDataProducer = new QuadrupedGlobalDataProducer(packetCommunicator);
   }
   
   private void createRuntimeEnvironment()
   {
      runtimeEnvironment = new QuadrupedRuntimeEnvironment(controlDT, timestampYoVariable, fullRobotModel, robotYoVariableRegistry,
                                                           yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead, globalDataProducer, footSwitches);
   }
   
   public QuadrupedControllerManager createControllerManager() throws IOException
   {
      createContactableFeet();
      createFootSwitches();
      createPacketCommunicator();
      createGlobalDataProducer();
      createRuntimeEnvironment();
      
      switch (controlMode)
      {
      case FORCE:
         return new QuadrupedForceControllerManager(runtimeEnvironment, physicalProperties);
      case FORCE_DEV:
         return new QuadrupedForceDevelopmentControllerManager(runtimeEnvironment, physicalProperties);
      case POSITION:
         return null;
      case POSITION_DEV:
         return null;
      default:
         return null;
      }
   }
   
   // OPTIONS
   
   public void setControlDT(double controlDT)
   {
      this.controlDT = controlDT;
   }
   
   public void setGravity(double gravity)
   {
      this.gravity = gravity;
   }
   
   public void setRobotYoVariableRegistry(YoVariableRegistry robotYoVariableRegistry)
   {
      this.robotYoVariableRegistry = robotYoVariableRegistry;
   }
   
   public void setTimestampYoVariable(DoubleYoVariable timestampYoVariable)
   {
      this.timestampYoVariable = timestampYoVariable;
   }
   
   public void setFullRobotModel(SDFFullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }
   
   public void setYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
   }
   
   public void setYoGraphicsListRegistryForDetachedOverhead(YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {
      this.yoGraphicsListRegistryForDetachedOverhead = yoGraphicsListRegistryForDetachedOverhead;
   }
   
   public void setKryoNetClassList(IHMCCommunicationKryoNetClassList kryoNetClassList)
   {
      this.kryoNetClassList = kryoNetClassList;
   }
   
   public void setPhysicalProperties(QuadrupedPhysicalProperties physicalProperties)
   {
      this.physicalProperties = physicalProperties;
   }
   
   public void setReferenceFrames(QuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;
   }
   
   public void setControlMode(QuadrupedControlMode controlMode)
   {
      this.controlMode = controlMode;
   }
}
