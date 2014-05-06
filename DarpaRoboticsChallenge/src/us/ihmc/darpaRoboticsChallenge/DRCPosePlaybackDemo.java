package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.PolyvalentHighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.posePlayback.PosePlaybackPacket;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;

public class DRCPosePlaybackDemo
{
   private static final HighLevelState JOINT_PD_CONTROL = HighLevelState.JOINT_PD_CONTROL;
   
   private final HumanoidRobotSimulation<SDFRobot> drcSimulation;
   private final DRCController drcController;
   private final PolyvalentHighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;

   public DRCPosePlaybackDemo(DRCSimulatedRobotInterface robotInterface, DRCRobotInitialSetup<SDFRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
                                    DRCSCSInitialSetup scsInitialSetup, AutomaticSimulationRunner automaticSimulationRunner,
                                    double timePerRecordTick, int simulationDataBufferSize, DRCRobotModel model)
   {
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(timePerRecordTick / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
      if (guiInitialSetup.isGuiShown())
         dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry(false);
      else
         dynamicGraphicObjectsListRegistry = null;
      YoVariableRegistry registry = new YoVariableRegistry("adjustableParabolicTrajectoryDemoSimRegistry");
      
      WalkingControllerParameters walkingControlParameters = model.getWalkingControlParameters();
      ArmControllerParameters armControlParameters = model.getArmControllerParameters();

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulation(walkingControlParameters);
      
      highLevelHumanoidControllerFactory = new PolyvalentHighLevelHumanoidControllerFactory(null,
            footstepTimingParameters, walkingControlParameters, walkingControlParameters, armControlParameters, false, false, false, JOINT_PD_CONTROL);

      SideDependentList<String> footForceSensorNames = model.getSensorInformation().getFeetForceSensorNames();
      
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, DRCConfigParameters.contactTresholdForceForSCS, footForceSensorNames);
      Pair<HumanoidRobotSimulation<SDFRobot>, DRCController> humanoidSimulation = DRCSimulationFactory.createSimulation(controllerFactory, null,
            robotInterface, robotInitialSetup, scsInitialSetup, guiInitialSetup, null, null, dynamicGraphicObjectsListRegistry, false,model);
      drcSimulation = humanoidSimulation.first();
      drcController = humanoidSimulation.second();

      // add other registries
      drcSimulation.addAdditionalYoVariableRegistriesToSCS(registry);
      
      SimulationConstructionSet simulationConstructionSet = drcSimulation.getSimulationConstructionSet();
      
      SDFRobot robot = drcSimulation.getRobot();
      //System.out.println(robot);
      HoldRobotInTheAir controller = new HoldRobotInTheAir(robot, simulationConstructionSet, drcController.getControllerModel());
      robot.setController(controller);
      controller.initialize();
      
      if (automaticSimulationRunner != null)
      {
         drcSimulation.start(automaticSimulationRunner);
      }
      else
      {
         drcSimulation.start(null);
      }
   }
   
   public FullRobotModel getControllerModel()
   {
      return drcController.getControllerModel();
   }
   
   public DRCController getDRCController()
   {
      return drcController;
   }
   
   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }
   
   public void setupPosePlaybackController(PosePlaybackPacket posePlaybackPacket, boolean transitionRequested)
   {
      highLevelHumanoidControllerFactory.createPosePlayBackController(posePlaybackPacket, true);
   }

   private class HoldRobotInTheAir implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      
      private final ArrayList<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      private final ArrayList<Vector3d> efp_offsetFromRootJoint = new ArrayList<>();
      private final double dx = 0.05, dy = 0.12, dz = 0.4;
      
      private final ArrayList<Vector3d> initialPositions = new ArrayList<>();

      private final DoubleYoVariable holdPelvisKp = new DoubleYoVariable("holdPelvisKp", registry);
      private final DoubleYoVariable holdPelvisKv = new DoubleYoVariable("holdPelvisKv", registry);
      private final DoubleYoVariable desiredHeight = new DoubleYoVariable("desiredHeight", registry);
      private final double robotMass, robotWeight;
      
      private final SDFRobot robot;
      
      private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry(true);
      private final ArrayList<DynamicGraphicPosition> efp_positionViz = new ArrayList<>();
      
      public HoldRobotInTheAir(SDFRobot robot, SimulationConstructionSet scs, SDFFullRobotModel sdfFullRobotModel)
      {
         this.robot = robot;
         robotMass = robot.computeCenterOfMass(new Point3d());
         robotWeight = robotMass * Math.abs(robot.getGravityZ());
         
         Joint jointToAddExternalForcePoints;
         try
         {
            String lastSpineJointName = sdfFullRobotModel.getChest().getParentJoint().getName();
            jointToAddExternalForcePoints = robot.getJoint(lastSpineJointName);
         }
         catch(NullPointerException e)
         {
            System.err.println("No chest or spine found. Stack trace:");
            e.printStackTrace();
            
            jointToAddExternalForcePoints = robot.getPelvisJoint();
         }
         
         holdPelvisKp.set(5000.0);
         holdPelvisKv.set(GainCalculator.computeDampingForSecondOrderSystem(robotMass, holdPelvisKp.getDoubleValue(), 1.0));

         efp_offsetFromRootJoint.add(new Vector3d(dx, dy, dz));
         efp_offsetFromRootJoint.add(new Vector3d(dx, -dy, dz));
         efp_offsetFromRootJoint.add(new Vector3d(-dx, dy, dz));
         efp_offsetFromRootJoint.add(new Vector3d(-dx, -dy, dz));
         
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            initialPositions.add(new Vector3d());
            
            String linkName = jointToAddExternalForcePoints.getLink().getName();
            ExternalForcePoint efp = new ExternalForcePoint("efp_" + linkName + "_" + String.valueOf(i) + "_", efp_offsetFromRootJoint.get(i), robot.getRobotsYoVariableRegistry());
            externalForcePoints.add(efp);
            jointToAddExternalForcePoints.addExternalForcePoint(efp);
            
            efp_positionViz.add(efp.getYoPosition().createDynamicGraphicPosition(efp.getName(), 0.05, YoAppearance.Red()));
         }
         
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("EFP", efp_positionViz);
         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
      }

      @Override
      public void initialize()
      {
         robot.update();
         desiredHeight.set(0.0);
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            externalForcePoints.get(i).getYoPosition().get(initialPositions.get(i));
            desiredHeight.add(initialPositions.get(i).z / initialPositions.size());
            efp_positionViz.get(i).update();
         }
         
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
            initialPositions.get(i).setZ(desiredHeight.getDoubleValue());
         
         doControl();
      }

      private final Vector3d proportionalTerm = new Vector3d();
      private final Vector3d derivativeTerm = new Vector3d();
      private final Vector3d pdControlOutput = new Vector3d();
      
      @Override
      public void doControl()
      {
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            initialPositions.get(i).setZ(desiredHeight.getDoubleValue());
            
            ExternalForcePoint efp = externalForcePoints.get(i);
            efp.getYoPosition().get(proportionalTerm);
            proportionalTerm.sub(initialPositions.get(i));
            proportionalTerm.scale(-holdPelvisKp.getDoubleValue());
//            proportionalTerm.setZ(Math.max(proportionalTerm.getZ(), 0.0));
            
            efp.getYoVelocity().get(derivativeTerm);
            derivativeTerm.scale(- holdPelvisKv.getDoubleValue());
            
            pdControlOutput.add(proportionalTerm, derivativeTerm);
            
            efp.setForce(pdControlOutput);
            efp.fz.add(robotWeight / efp_offsetFromRootJoint.size());

            efp_positionViz.get(i).update();
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
         return registry.getName();
      }

      @Override
      public String getDescription()
      {
         return getName();
      }
   }
}
