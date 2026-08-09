package us.ihmc.exampleSimulations.beetle;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.exampleSimulations.beetle.controller.HexapodSimulationController;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleInverseDynamicsParameters;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleModelFactory;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleVirtualModelControlParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.simulationConstructionSetTools.tools.TerrainObjectDefinitionTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RhinoBeetleSimpleSimulation
{
   private static final double SIMULATION_DT = 0.0001;
   private static final double CONTROLLER_DT = 0.001;
   private static final int CONTROL_TICKS_PER_SIMULATION_TICK = (int) Math.round(CONTROLLER_DT / SIMULATION_DT);

   public RhinoBeetleSimpleSimulation()
   {
      RhinoBeetleModelFactory modelFactory = new RhinoBeetleModelFactory();
      RobotDefinition robotDefinition = modelFactory.getRobotDefinition();
      FullRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      YoRegistry registry = new YoRegistry("RhinoBeetleParameters");
      ArrayList<String> jointsToControl = new ArrayList<>();

      RhinoBeetleInverseDynamicsParameters idParameters = new RhinoBeetleInverseDynamicsParameters(registry);
      RhinoBeetleVirtualModelControlParameters vmcParameters = new RhinoBeetleVirtualModelControlParameters(registry);

      HexapodSimulationController[] controllerHolder = new HexapodSimulationController[1];
      robotDefinition.addControllerDefinition((controllerInput, controllerOutput) -> controllerHolder[0] = new HexapodSimulationController(controllerInput,
                                                                                                                                             controllerOutput,
                                                                                                                                             fullRobotModel,
                                                                                                                                             jointsToControl,
                                                                                                                                             idParameters,
                                                                                                                                             vmcParameters,
                                                                                                                                             yoGraphicsListRegistry,
                                                                                                                                             CONTROLLER_DT,
                                                                                                                                             CONTROL_TICKS_PER_SIMULATION_TICK));

      // Unlike SCS1's LinearGroundContactModel, SCS2's ContactPointBasedForceCalculator computes the
      // normal force as Kz * penetration / (stiffeningLength - penetration), ramping to a hard wall as
      // penetration approaches stiffeningLength (falling back to a fixed 0.002 denominator once within
      // 2mm of it). stiffeningLength has no SCS1 analog and defaults to 0.0 when unset, which makes that
      // denominator negative for any real penetration - so the model always took the hard-wall branch,
      // Kz * penetration / 0.002 (an effective stiffness of Kz * 500 from the first millimeter of contact),
      // which is what was driving the leg joints unstable.
      //
      // stiffeningLength should be picked as a genuine physical bound on how far a foot may sink before
      // the ground goes rigid, sized to the robot: total mass here is ~9.5kg (5kg body + 18 * 0.25kg
      // links, see model.sdf), so on a tripod stance each planted foot carries ~31N. Solving the engine's
      // static equilibrium Kz * z / (L - z) = 31N for z with Kz=200 and L=0.015 gives ~2mm of sink at
      // rest - a reasonable soft-ground look with 7x headroom below L before hitting the hard-wall regime,
      // enough to absorb footstep-impact transients without saturating every step.
      ContactPointBasedContactParameters contactParameters = new ContactPointBasedContactParameters();
      contactParameters.setKz(200.0);
      contactParameters.setBz(250.0);
      contactParameters.setKxy(5000.0);
      contactParameters.setBxy(100.0);
      contactParameters.setStiffeningLength(0.015);

      SimulationConstructionSet2 scs = new SimulationConstructionSet2("RhinoBeetle",
                                                                        PhysicsEngineFactory.newContactPointBasedPhysicsEngineFactory(contactParameters));
      scs.getSimulationSession().setGravity(0.0, 0.0, -9.81);
      scs.addTerrainObject(flatGround());
      scs.addRobot(robotDefinition);
      scs.getRootRegistry().addChild(registry);
      scs.setDT(SIMULATION_DT);

      HexapodSimulationController controller = controllerHolder[0];
      scs.addYoGraphic(controller.getSCS2YoGraphics());
      scs.showOverheadPlotter2D(true);
      scs.requestPlotter2DCoordinateTracking(controller.getBodyPosition2D().getYoX().getName(),
                                              controller.getBodyPosition2D().getYoY().getName(),
                                              ReferenceFrame.getWorldFrame().getName());

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);

      scs.startSimulationThread();
      scs.play();

      // TEMP DEBUG: inject an extra, very bright light directly into the live scene to test whether
      // the default scene lighting is actually reaching materials. Remove before committing.
      new java.util.Timer().schedule(new java.util.TimerTask()
      {
         @Override
         public void run()
         {
            javafx.application.Platform.runLater(() ->
            {
               try
               {
                  javafx.scene.SubScene scene3D = scs.getSessionVisualizerToolkit().getMainScene3D();
                  javafx.scene.Parent root = scene3D.getRoot();
                  if (root instanceof javafx.scene.Group)
                  {
                     javafx.scene.AmbientLight boost = new javafx.scene.AmbientLight(javafx.scene.paint.Color.WHITE);
                     ((javafx.scene.Group) root).getChildren().add(boost);
                     PrintTools.info("DEBUG: injected full-white AmbientLight into scene root");
                  }
                  else
                  {
                     PrintTools.info("DEBUG: scene root is not a Group, it is " + root.getClass());
                  }
               }
               catch (Exception e)
               {
                  e.printStackTrace();
               }
            });
         }
      }, 6000);
   }

   /**
    * Same flat-ground terrain and SCS1-to-SCS2 conversion path used by ZuluFlatGroundWalkingTrack
    * (via SCS2AvatarSimulationFactory.setCommonAvatarEnvrionmentInterface(...)), called directly here
    * without pulling in the full avatar simulation factory.
    */
   private static TerrainObjectDefinition flatGround()
   {
      return TerrainObjectDefinitionTools.toTerrainObjectDefinition(new FlatGroundEnvironment());
   }

   public static void main(String[] args)
   {
      RhinoBeetleSimpleSimulation simulationFactory = new RhinoBeetleSimpleSimulation();
   }
}
