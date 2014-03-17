package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.visualization.SliderBoardFactory;
import us.ihmc.atlas.visualization.WalkControllerSliderBoard;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.PolyvalentHighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.PlainDRCRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCSimDRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.valkyrie.ValkyrieInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCRobotInverseDynamicsController
{
   private static final double ROBOT_FLOATING_HEIGHT = 0.3;

   private static final HighLevelState INVERSE_DYNAMICS_JOINT_CONTROL = HighLevelState.INVERSE_DYNAMICS_JOINT_CONTROL;
   
   private final HumanoidRobotSimulation<SDFRobot> drcSimulation;
   private final DRCController drcController;

   public DRCRobotInverseDynamicsController(WalkingControllerParameters drcControlParameters, ArmControllerParameters armControllerParameters, 
                                    DRCRobotInterface robotInterface, DRCRobotInitialSetup<SDFRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
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

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulation(drcControlParameters);
      
      PolyvalentHighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory = new PolyvalentHighLevelHumanoidControllerFactory(null,
            footstepTimingParameters, drcControlParameters, drcControlParameters, armControllerParameters, false, false, false, INVERSE_DYNAMICS_JOINT_CONTROL);

      SideDependentList<String> footForceSensorNames = new SideDependentList<>();
      for(RobotSide robotSide : RobotSide.values)
      {
         footForceSensorNames.put(robotSide, robotInterface.getJointMap().getJointBeforeFootName(robotSide));
      }
      
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, DRCConfigParameters.contactTresholdForceForSCS, footForceSensorNames);
      Pair<HumanoidRobotSimulation<SDFRobot>, DRCController> humanoidSimulation = DRCSimulationFactory.createSimulation(controllerFactory, null,
            robotInterface, robotInitialSetup, scsInitialSetup, guiInitialSetup, null, null, dynamicGraphicObjectsListRegistry, model);
      drcSimulation = humanoidSimulation.first();
      drcController = humanoidSimulation.second();

      // add other registries
      drcSimulation.addAdditionalYoVariableRegistriesToSCS(registry);
      
      
      HoldRobotInTheAir controller = new HoldRobotInTheAir(drcSimulation.getRobot());
      drcSimulation.getRobot().setController(controller);
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

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }

   public DRCController getDrcController()
   {
      return drcController;
   }

   public static void main(String[] args) throws JSAPException
   {
      // Add flag to set robot model
      JSAP jsap = new JSAP();
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + Arrays.toString(DRCRobotModel.values()));
      
      DRCRobotModel model;
      try
      {
         jsap.registerParameter(robotModel);

         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            model = DRCRobotModel.valueOf(config.getString("robotModel"));
         }
         else
         {
            System.out.println("Enter a robot model.");
            return;
         }
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
         return;
      }
      
      AutomaticSimulationRunner automaticSimulationRunner = null;

      SliderBoardFactory sliderBoardFactory = WalkControllerSliderBoard.getFactory();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, sliderBoardFactory);

      DRCRobotInterface robotInterface = new PlainDRCRobot(model, false);
      
      final double groundHeight = 0.0;
      GroundProfile groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotInterface.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup;
      if (model.equals(DRCRobotModel.VALKYRIE))
      {
         robotInitialSetup = new ValkyrieInitialSetup(groundHeight + ROBOT_FLOATING_HEIGHT, initialYaw);
      }
      else
      {
         robotInitialSetup = new DRCSimDRCRobotInitialSetup(groundHeight + ROBOT_FLOATING_HEIGHT, initialYaw);
      }

      WalkingControllerParameters drcControlParameters = model.getWalkingControlParamaters();
      ArmControllerParameters armControlParameters = model.getArmControllerParameters();
      
      new DRCRobotInverseDynamicsController(drcControlParameters, armControlParameters, robotInterface, robotInitialSetup, guiInitialSetup, scsInitialSetup,
                                    automaticSimulationRunner, DRCConfigParameters.CONTROL_DT, 16000, model);
   }
   
   private class HoldRobotInTheAir implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      
      private final ArrayList<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      private final ArrayList<Vector3d> efp_offsetFromRootJoint = new ArrayList<>();
      private final double dx = 0.5, dy = 0.5;
      
      private final ArrayList<Vector3d> initialPositions = new ArrayList<>();

      private final DoubleYoVariable holdPelvisKp = new DoubleYoVariable("holdPelvisKp", registry);
      private final DoubleYoVariable holdPelvisKv = new DoubleYoVariable("holdPelvisKv", registry);
      private final double robotMass, robotWeight;
      
      private final SDFRobot robot;
      
      public HoldRobotInTheAir(SDFRobot robot)
      {
         this.robot = robot;
         robotMass = robot.computeCenterOfMass(new Point3d());
         robotWeight = robotMass * Math.abs(robot.getGravityZ());
         
         holdPelvisKp.set(1500.0);
         holdPelvisKv.set(GainCalculator.computeDampingForSecondOrderSystem(robotMass, holdPelvisKp.getDoubleValue(), 1.0));
         
         efp_offsetFromRootJoint.add(new Vector3d(dx, dy, 0.0));
         efp_offsetFromRootJoint.add(new Vector3d(dx, -dy, 0.0));
         efp_offsetFromRootJoint.add(new Vector3d(-dx, dy, 0.0));
         efp_offsetFromRootJoint.add(new Vector3d(-dx, -dy, 0.0));
         
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            initialPositions.add(new Vector3d());
            
            ExternalForcePoint efp = new ExternalForcePoint("efp_pelvis_" + String.valueOf(i) + "_", efp_offsetFromRootJoint.get(i), robot);
            externalForcePoints.add(efp);
            robot.getPelvisJoint().addExternalForcePoint(efp);
         }
      }

      @Override
      public void initialize()
      {
         robot.update();
         
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            externalForcePoints.get(i).getYoPosition().getPoint3d(initialPositions.get(i));
         }
         
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
            ExternalForcePoint efp = externalForcePoints.get(i);
            efp.getYoPosition().getPoint3d(proportionalTerm);
            proportionalTerm.sub(initialPositions.get(i));
            proportionalTerm.scale(-holdPelvisKp.getDoubleValue());
            
            efp.getYoVelocity().get(derivativeTerm);
            derivativeTerm.scale(- holdPelvisKv.getDoubleValue());
            
            pdControlOutput.add(proportionalTerm, derivativeTerm);
            
            efp.setForce(pdControlOutput);
            efp.fz.add(robotWeight / efp_offsetFromRootJoint.size());
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
