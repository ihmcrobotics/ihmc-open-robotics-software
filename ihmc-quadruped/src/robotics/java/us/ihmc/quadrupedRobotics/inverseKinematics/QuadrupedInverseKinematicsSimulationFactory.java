package us.ihmc.quadrupedRobotics.inverseKinematics;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationController;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

import java.io.IOException;

public class QuadrupedInverseKinematicsSimulationFactory
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private final RequiredFactoryField<QuadrupedModelFactory> modelFactory = new RequiredFactoryField<>("modelFactory");
   private final RequiredFactoryField<FloatingRootJointRobot> sdfRobot = new RequiredFactoryField<>("sdfRobot");
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<JointDesiredOutputList> jointDesiredOutputList = new RequiredFactoryField<>("jointDesiredOutputList");
   private final RequiredFactoryField<Double> simulationDT = new RequiredFactoryField<>("simulationDT");
   private final RequiredFactoryField<Double> controlDT = new RequiredFactoryField<>("controlDT");
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<OutputWriter> outputWriter = new RequiredFactoryField<>("outputWriter");
   private final RequiredFactoryField<QuadrupedInitialPositionParameters> initialPositionParameters = new RequiredFactoryField<>("initialPositionParameters");
   private final RequiredFactoryField<RobotQuadrant[]> quadrants = new RequiredFactoryField<>("quadrants");
   private final RequiredFactoryField<ControllerCoreOptimizationSettings> optimizationSettings = new RequiredFactoryField<>("optimizationSettings");

   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private SensorReader sensorReader;
   private QuadrupedInverseKinematicsController ikController;
   private QuadrupedSimulationController simulationController;

   private void setupYoRegistries()
   {
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
   }

   private void createSensorReader()
   {
      sensorReader = new SDFQuadrupedPerfectSimulatedSensor(quadrants.get(), sdfRobot.get(), fullRobotModel.get(), null, null);
   }

   private void createKinematicsController()
   {
      ikController = new QuadrupedInverseKinematicsController(fullRobotModel.get(), quadrants.get(), jointDesiredOutputList.get(),
                                                              optimizationSettings.get(), controlDT.get(), gravity.get(), yoGraphicsListRegistry);
   }

   private void createSimulationController()
   {
      simulationController = new QuadrupedSimulationController(sdfRobot.get(), sensorReader, outputWriter.get(), ikController, null, null, null);
   }

   private void setupSDFRobot()
   {
      int simulationTicksPerControllerTick = (int) Math.round(controlDT.get() / simulationDT.get());
      sdfRobot.get().setController(simulationController, simulationTicksPerControllerTick);
      sdfRobot.get().setPositionInWorld(initialPositionParameters.get().getInitialBodyPosition());
      sdfRobot.get().setOrientation(initialPositionParameters.get().getInitialBodyOrientation());

      for (QuadrupedJointName quadrupedJointName : modelFactory.get().getQuadrupedJointNames())
      {
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = sdfRobot.get()
                                                                   .getOneDegreeOfFreedomJoint(modelFactory.get().getSDFNameForJointName(quadrupedJointName));
         if (oneDegreeOfFreedomJoint != null)
         {
            oneDegreeOfFreedomJoint.setQ(initialPositionParameters.get().getInitialJointPosition(quadrupedJointName));
         }
      }
      try
      {
         sdfRobot.get().update();
         sdfRobot.get().doDynamicsButDoNotIntegrate();
         sdfRobot.get().update();
      }
      catch (UnreasonableAccelerationException unreasonableAccelerationException)
      {
         throw new RuntimeException("UnreasonableAccelerationException");
      }

      Point3D initialCoMPosition = new Point3D();
      double totalMass = sdfRobot.get().computeCenterOfMass(initialCoMPosition);

      sdfRobot.get().setGravity(gravity.get());
      PrintTools.info(this, sdfRobot.get().getName() + " total mass: " + totalMass);
   }

   public SimulationConstructionSet createSimulation() throws IOException
   {

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      setupYoRegistries();
      createSensorReader();
      createKinematicsController();

      createSimulationController();
      setupSDFRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot.get(), simulationTestingParameters);
      scs.setGroundVisible(false);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(simulationDT.get(), (int) (0.01 / simulationDT.get()));
      if (scs.getSimulationConstructionSetParameters().getCreateGUI())
      {
         scs.setCameraTrackingVars("q_x", "q_y", "q_z");
         scs.setCameraDollyVars("q_x", "q_y", "q_z");
         scs.setCameraDollyOffsets(4.0, 4.0, 1.0);
         SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
         simulationOverheadPlotterFactory.setVariableNameToTrack("centerOfMass");
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
         simulationOverheadPlotterFactory.createOverheadPlotter();
      }

      FactoryTools.disposeFactory(this);

      return scs;
   }

   public void setModelFactory(QuadrupedModelFactory modelFactory)
   {
      this.modelFactory.set(modelFactory);
   }

   public void setSdfRobot(FloatingRootJointRobot sdfRobot)
   {
      this.sdfRobot.set(sdfRobot);
   }

   public void setFullRobotModel(FullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setJointDesiredOutputList(JointDesiredOutputList jointDesiredOutputList)
   {
      this.jointDesiredOutputList.set(jointDesiredOutputList);
   }

   public void setSimulationDT(double simulationDT)
   {
      this.simulationDT.set(simulationDT);
   }

   public void setControlDT(double controlDT)
   {
      this.controlDT.set(controlDT);
   }

   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }

   public void setOutputWriter(OutputWriter outputWriter)
   {
      this.outputWriter.set(outputWriter);
   }

   public void setInitialPositionParameters(QuadrupedInitialPositionParameters initialPositionParameters)
   {
      this.initialPositionParameters.set(initialPositionParameters);
   }

   public void setQuadrants(RobotQuadrant[] quadrants)
   {
      this.quadrants.set(quadrants);
   }

   public void setOptimizationSettings(ControllerCoreOptimizationSettings optimizationSettings)
   {
      this.optimizationSettings.set(optimizationSettings);
   }
}
