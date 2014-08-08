package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelPositionControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.SideDependentList;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.controller.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;

public class DRCHighLevelPositionControlDemo
{
   private final DRCSimulationFactory drcSimulation;


   public DRCHighLevelPositionControlDemo(DRCRobotInitialSetup<SDFRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
                                    DRCSCSInitialSetup scsInitialSetup, DRCRobotModel model)
   {
      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      ArmControllerParameters armControllerParameters = model.getArmControllerParameters();
      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(model.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulation(walkingControllerParameters);

      ContactableBodiesFactory contactableBodiesFactory = model.getContactPointParameters().getContactableBodiesFactory();

      SideDependentList<String> footForceSensorNames = model.getSensorInformation().getFeetForceSensorNames();

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory,
            footForceSensorNames, footstepTimingParameters, walkingControllerParameters, armControllerParameters,
            false, false, HighLevelState.DO_NOTHING_BEHAVIOR);
      controllerFactory.addHighLevelBehaviorFactory(new HighLevelPositionControllerFactory(true));

      drcSimulation = new DRCSimulationFactory(model, controllerFactory, null, robotInitialSetup, scsInitialSetup, guiInitialSetup, null);

      HoldRobotInTheAir controller = new HoldRobotInTheAir(drcSimulation.getRobot(), drcSimulation.getSimulationConstructionSet(), model.createFullRobotModel());
      drcSimulation.getRobot().setController(controller);
      controller.initialize();

      drcSimulation.start();
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }
   
   private class HoldRobotInTheAir implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      
      private final ArrayList<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      private final ArrayList<Vector3d> efp_offsetFromRootJoint = new ArrayList<>();
      private final double dx = 0.0, dy = 0.12, dz = 0.4;
      
      private final ArrayList<Vector3d> initialPositions = new ArrayList<>();

      private final DoubleYoVariable holdPelvisKp = new DoubleYoVariable("holdPelvisKp", registry);
      private final DoubleYoVariable holdPelvisKv = new DoubleYoVariable("holdPelvisKv", registry);
      private final double robotMass, robotWeight;
      
      private final SDFRobot robot;
      
      private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
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
         
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            externalForcePoints.get(i).getYoPosition().get(initialPositions.get(i));
            efp_positionViz.get(i).update();
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
            efp.getYoPosition().get(proportionalTerm);
            proportionalTerm.sub(initialPositions.get(i));
            proportionalTerm.scale(-holdPelvisKp.getDoubleValue());
            
            efp.getYoVelocity().get(derivativeTerm);
            derivativeTerm.scale(- holdPelvisKv.getDoubleValue());
            
            pdControlOutput.add(proportionalTerm, derivativeTerm);
            
            pdControlOutput.setZ(pdControlOutput.getZ() + robotWeight / efp_offsetFromRootJoint.size());
            efp.setForce(pdControlOutput);

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
