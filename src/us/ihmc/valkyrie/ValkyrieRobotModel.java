package us.ihmc.valkyrie;

import java.io.InputStream;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.valkyrie.io.ValkyrieOutputWriterWithAccelerationIntegration;
import us.ihmc.valkyrie.models.ModelRoot;
import us.ihmc.valkyrie.paramaters.ValkyrieArmControllerParameters;
import us.ihmc.valkyrie.paramaters.ValkyrieContactPointParamaters;
import us.ihmc.valkyrie.paramaters.ValkyrieJointMap;
import us.ihmc.valkyrie.paramaters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.paramaters.ValkyrieSensorInformation;
import us.ihmc.valkyrie.paramaters.ValkyrieStateEstimatorParameters;
import us.ihmc.valkyrie.paramaters.ValkyrieWalkingControllerParameters;

import com.jme3.math.Transform;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;

public class ValkyrieRobotModel implements DRCRobotModel
{
   private static Class<ModelRoot> valModelRoot = ModelRoot.class;
   private final ArmControllerParameters armControllerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final DRCRobotContactPointParamaters contactPointParamaters;
   private StateEstimatorParameters stateEstimatorParamaters;
   private final DRCRobotPhysicalProperties physicalProperties;
   private DRCRobotSensorInformation sensorInformation;
   private final DRCRobotJointMap jointMap;
   private final String robotName = "VALKYRIE";
   private final String sdfFileName = "V1/sdf/V1_sim.sdf";
   
   
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
      this.walkingControllerParameters = new ValkyrieWalkingControllerParameters(runningOnRealRobot);
      this.contactPointParamaters = new ValkyrieContactPointParamaters(jointMap);
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
         
         // FIXME: For now, the sim cannot simulate off-axis force sensors 
         if(runningOnRealRobot)
         {
            if(forceSensorNames.equals("LeftAnkle"))
            {
               transform.set(ValkyrieSensorInformation.transformFromAnkleURDFFrameToZUpFrames.get(RobotSide.LEFT));
            }
            else if(forceSensorNames.equals("RightAnkle"))
            {
               transform.set(ValkyrieSensorInformation.transformFromAnkleURDFFrameToZUpFrames.get(RobotSide.RIGHT));               
            }
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
      return new Transform();
   }

   private String getSdfFile()
   {
      return sdfFileName;
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
   public DRCRobotContactPointParamaters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return contactPointParamaters;
   }

   @Override
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter valkyrieOutputWriter, boolean runningOnRealRobot)
   {
      ValkyrieOutputWriterWithAccelerationIntegration valkyrieOutputWriterWithAccelerationIntegration =
            new ValkyrieOutputWriterWithAccelerationIntegration(valkyrieOutputWriter, getControllerDT(), runningOnRealRobot);

      valkyrieOutputWriterWithAccelerationIntegration.setAlphaDesiredVelocity(0.98);
      valkyrieOutputWriterWithAccelerationIntegration.setAlphaDesiredPosition(0.0);
      valkyrieOutputWriterWithAccelerationIntegration.setVelocityGains(15.0);
      valkyrieOutputWriterWithAccelerationIntegration.setPositionGains(0.0);

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
   public JaxbSDFLoader getJaxbSDFLoader()
   {
      
      return loader;
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
      return loader.createRobot(jointMap, createCollisionMeshes);
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

}
