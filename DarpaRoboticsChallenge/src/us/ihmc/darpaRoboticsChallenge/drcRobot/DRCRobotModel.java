package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.net.URI;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.util.RobotNetworkParameters;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandType;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.ihmcPerception.footstepPlanner.FootstepParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.physics.ScsCollisionConfigure;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.DRCRobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControlParameters;

import com.jme3.math.Transform;

public interface DRCRobotModel extends WholeBodyControlParameters
{
   //TODO: RobotBoundingBoxes.java

//   public abstract boolean isRunningOnRealRobot();
   
   public abstract FootstepParameters getFootstepParameters();

   public abstract WalkingControllerParameters getDrivingControllerParameters();

   public abstract StateEstimatorParameters getStateEstimatorParameters();

   public abstract DRCRobotPhysicalProperties getPhysicalProperties();

   public abstract DRCRobotJointMap getJointMap();

   public abstract DRCRobotSensorInformation getSensorInformation();

   public abstract DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw);

   public abstract ScsCollisionConfigure getPhysicsConfigure(SDFRobot robotModel);

   public abstract void setJointDamping(SDFRobot simulatedRobot);

   public abstract HandModel getHandModel();

   public abstract Transform getOffsetHandFromWrist(RobotSide side);

   public abstract SDFFullRobotModel createFullRobotModel();

   public abstract SDFRobot createSdfRobot(boolean createCollisionMeshes);

   public abstract double getSimulateDT();

   public abstract double getEstimatorDT();

   public abstract GeneralizedSDFRobotModel getGeneralizedRobotModel();

   public abstract PPSTimestampOffsetProvider getPPSTimestampOffsetProvider();

   public abstract DRCSensorSuiteManager getSensorSuiteManager(URI rosCoreURI);
   
   public abstract RobotNetworkParameters getNetworkParameters();
   
   public abstract HandCommandManager createHandCommandManager(AbstractNetworkProcessorNetworkingManager networkManager);
   
   public abstract MultiThreadedRobotControlElement createSimulatedHandController(SDFRobot simulatedRobot, ThreadDataSynchronizer threadDataSynchronizer, GlobalDataProducer globalDataProducer);

   public abstract DRCHandType getDRCHandType();
}
