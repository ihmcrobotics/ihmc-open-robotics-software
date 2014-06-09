package us.ihmc.valkyrie;

import java.io.InputStream;
import java.net.URI;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrie.io.ValkyrieOutputWriterWithAccelerationIntegration;
import us.ihmc.valkyrie.models.ModelRoot;
import us.ihmc.valkyrie.paramaters.ValkyrieArmControllerParameters;
import us.ihmc.valkyrie.paramaters.ValkyrieContactPointParameters;
import us.ihmc.valkyrie.paramaters.ValkyrieJointMap;
import us.ihmc.valkyrie.paramaters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.paramaters.ValkyrieSensorInformation;
import us.ihmc.valkyrie.paramaters.ValkyrieStateEstimatorParameters;
import us.ihmc.valkyrie.paramaters.ValkyrieWalkingControllerParameters;
import us.ihmc.valkyrie.sensors.ValkyrieSensorSuiteManager;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;

public class ValkyrieRobotModel implements DRCRobotModel
{
   private static final boolean PRINT_MODEL = false;
   
   private static Class<ModelRoot> valModelRoot = ModelRoot.class;
   private final ArmControllerParameters armControllerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final DRCRobotContactPointParameters contactPointParamaters;
   private StateEstimatorParameters stateEstimatorParamaters;
   private final DRCRobotPhysicalProperties physicalProperties;
   private DRCRobotSensorInformation sensorInformation;
   private final DRCRobotJointMap jointMap;
   private final String robotName = "VALKYRIE";
   private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>();

   
   private final String[] resourceDirectories = {
         valModelRoot.getResource("").getFile(),
         valModelRoot.getResource("V1/").getFile(),
         valModelRoot.getResource("V1/sdf/").getFile(),
         valModelRoot.getResource("V1/meshes/").getFile(),
         valModelRoot.getResource("V1/meshes/2013_05_16/").getFile(),
         };
   
   private final JaxbSDFLoader loader;
   private final boolean runningOnRealRobot;
   
   public ValkyrieRobotModel(boolean runningOnRealRobot, boolean headless)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = new ValkyrieJointMap();
      this.physicalProperties = new ValkyriePhysicalProperties();
      this.sensorInformation = new ValkyrieSensorInformation();
      this.armControllerParameters = new ValkyrieArmControllerParameters(runningOnRealRobot);
      this.walkingControllerParameters = new ValkyrieWalkingControllerParameters(jointMap, runningOnRealRobot);
      this.contactPointParamaters = new ValkyrieContactPointParameters(jointMap);
      if(headless)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[]{}, getSdfFileAsStream(), true);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), false);
      }
      
      SDFJointNameMap jointMap = getJointMap();
      for(String forceSensorNames : ValkyrieSensorInformation.forceSensorNames)
      {
         Transform3D transform = new Transform3D();
         if(forceSensorNames.equals("LeftAnkle"))
         {
            transform.set(ValkyrieSensorInformation.transformFromMeasurementToAnkleZUpFrames.get(RobotSide.LEFT));
         }
         else if(forceSensorNames.equals("RightAnkle"))
         {
            transform.set(ValkyrieSensorInformation.transformFromMeasurementToAnkleZUpFrames.get(RobotSide.RIGHT));               
         }
            
         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, transform);
      }
      
   }
   
   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return armControllerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControlParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      if(stateEstimatorParamaters == null)
      {
         stateEstimatorParamaters = new ValkyrieStateEstimatorParameters(runningOnRealRobot, getEstimatorDT());
      }
      return stateEstimatorParamaters;
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return physicalProperties;
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return jointMap;
   }

   @Override
   public Transform getOffsetHandFromWrist(RobotSide side)
   {
//      if (offsetHandFromWrist.get(side) == null)
//      {
         createTransforms();
//      }

      return offsetHandFromWrist.get(side);
   }
   
   private void createTransforms()
   {
      for(RobotSide robotSide : RobotSide.values())
      {
         Vector3f centerOfHandToWristTranslation = new Vector3f();
         float[] angles = new float[3];

            centerOfHandToWristTranslation = new Vector3f(0f, (float) robotSide.negateIfLeftSide(0.015f), -0.06f);
            angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
            angles[1] = 0.0f;   
            angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
//       
         Quaternion centerOfHandToWristRotation = new Quaternion(angles);
         offsetHandFromWrist.set(robotSide, new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation));
      }
   }

   private String getSdfFile()
   {
      return ValkyrieConfigurationRoot.SDF_FILE;
   }

   private String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   private InputStream getSdfFileAsStream()
   {
      return valModelRoot.getResourceAsStream(getSdfFile());
   }

   @Override
   public String toString()
   {
      return robotName;
   }

   @Override
   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new ValkyrieInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure(SDFRobot robotModel)
   {
      return null;
   }

   @Override
   public DRCRobotContactPointParameters getContactPointParameters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return contactPointParamaters;
   }

   @Override
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter valkyrieOutputWriter, boolean runningOnRealRobot)
   {
      ValkyrieOutputWriterWithAccelerationIntegration valkyrieOutputWriterWithAccelerationIntegration =
            new ValkyrieOutputWriterWithAccelerationIntegration(valkyrieOutputWriter, getControllerDT(), runningOnRealRobot);

      valkyrieOutputWriterWithAccelerationIntegration.setAlphaDesiredVelocity(0.98, 0.0);
      valkyrieOutputWriterWithAccelerationIntegration.setAlphaDesiredPosition(0.0, 0.0);
      valkyrieOutputWriterWithAccelerationIntegration.setVelocityGains(15.0, 0.0);
      valkyrieOutputWriterWithAccelerationIntegration.setPositionGains(0.0, 0.0);

      return valkyrieOutputWriterWithAccelerationIntegration;
   }

   //For Sim Only
   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      System.err.println("Joint Damping not setup for Valkyrie. ValkyrieRobotModel setJointDamping!");
   }

   @Override
   public HandModel getHandModel()
   {
	   return null;
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return getWalkingControlParameters();
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public SDFFullRobotModel createFullRobotModel()
   {
      return loader.createFullRobotModel(getJointMap());
   }

   @Override
   public SDFRobot createSdfRobot(boolean createCollisionMeshes)
   {
      SDFRobot sdfRobot = loader.createRobot(jointMap, createCollisionMeshes);

      if (PRINT_MODEL) 
      {
         System.out.println("\nValkyrieRobotModel Link Masses:");

         StringBuffer stringBuffer = new StringBuffer();
         sdfRobot.printRobotJointsAndMasses(stringBuffer);
         System.out.println(stringBuffer);
         System.out.println();

         System.out.println("ValkyrieRobotModel: \n" + sdfRobot);
      }

      return sdfRobot;
   }
   
   @Override
   public double getSimulateDT()
   {
      return 0.0001;
   }

   @Override
   public double getEstimatorDT()
   {
      return 0.002;  
   }

   @Override
   public double getControllerDT()
   {
      return 0.006;
   }
   

   @Override
   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public PPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      return new AlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager(URI rosCoreURI)
   {
      return new ValkyrieSensorSuiteManager(rosCoreURI, getPPSTimestampOffsetProvider(), sensorInformation);
   }

}
