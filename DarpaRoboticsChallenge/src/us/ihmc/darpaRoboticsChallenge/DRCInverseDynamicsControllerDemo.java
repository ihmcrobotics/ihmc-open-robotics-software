package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InverseDynamicsJointControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.InverseDynamicsJointController;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class DRCInverseDynamicsControllerDemo
{
   private final DRCSimulationFactory drcSimulation;

   public DRCInverseDynamicsControllerDemo(DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
         DRCSCSInitialSetup scsInitialSetup, DRCRobotModel model)
   {
      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(model.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = model.getCapturePointPlannerParameters();
      ArmControllerParameters armControllerParameters = model.getArmControllerParameters();

      ContactableBodiesFactory contactableBodiesFactory = model.getContactPointParameters().getContactableBodiesFactory();

      DRCRobotSensorInformation sensorInformation = model.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters,
            HighLevelState.DO_NOTHING_BEHAVIOR);
      controllerFactory.addHighLevelBehaviorFactory(new InverseDynamicsJointControllerFactory(true));

      drcSimulation = new DRCSimulationFactory(model, controllerFactory, null, robotInitialSetup, scsInitialSetup, guiInitialSetup, null);

      SimulationConstructionSet simulationConstructionSet = drcSimulation.getSimulationConstructionSet();

      HoldRobotInTheAir controller = new HoldRobotInTheAir(drcSimulation.getRobot(), simulationConstructionSet, model.createFullRobotModel());
      drcSimulation.getRobot().setController(controller);
      controller.initialize();

      new InverseDynamicsJointController.GravityCompensationSliderBoard(simulationConstructionSet, model.createFullRobotModel(),
            simulationConstructionSet.getRootRegistry(), "desiredHeight", 0.5, 2.0);

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
      private final double dx = 0.05, dy = 0.12, dz = 0.4;

      private final ArrayList<Vector3d> initialPositions = new ArrayList<>();

      private final DoubleYoVariable holdPelvisKp = new DoubleYoVariable("holdPelvisKp", registry);
      private final DoubleYoVariable holdPelvisKv = new DoubleYoVariable("holdPelvisKv", registry);
      private final DoubleYoVariable desiredHeight = new DoubleYoVariable("desiredHeight", registry);
      private final double robotMass, robotWeight;

      private final SDFRobot robot;

      private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      private final ArrayList<YoGraphicPosition> efp_positionViz = new ArrayList<>();

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
         catch (NullPointerException e)
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
            ExternalForcePoint efp = new ExternalForcePoint("efp_" + linkName + "_" + String.valueOf(i) + "_", efp_offsetFromRootJoint.get(i),
                  robot.getRobotsYoVariableRegistry());
            externalForcePoints.add(efp);
            jointToAddExternalForcePoints.addExternalForcePoint(efp);

            efp_positionViz.add(new YoGraphicPosition(efp.getName(), efp.getYoPosition(), 0.05, YoAppearance.Red()));
         }

         yoGraphicsListRegistry.registerYoGraphics("EFP", efp_positionViz);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
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
            proportionalTerm.setZ(Math.max(proportionalTerm.getZ(), 0.0));

            efp.getYoVelocity().get(derivativeTerm);
            derivativeTerm.scale(-holdPelvisKv.getDoubleValue());

            pdControlOutput.add(proportionalTerm, derivativeTerm);

            efp.setForce(pdControlOutput);
            //            efp.fz.add(robotWeight / efp_offsetFromRootJoint.size());

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
