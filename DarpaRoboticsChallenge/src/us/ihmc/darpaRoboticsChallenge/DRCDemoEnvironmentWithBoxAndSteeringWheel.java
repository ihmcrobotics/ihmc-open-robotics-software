package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;

public class DRCDemoEnvironmentWithBoxAndSteeringWheel implements CommonAvatarEnvironmentInterface
{
   private static final double BOX_LENGTH = 0.5;
   private static final double BOX_WIDTH = 0.5;
   private static final double BOX_HEIGHT = 0.5;
   private final CombinedTerrainObject combinedTerrainObject;

   public DRCDemoEnvironmentWithBoxAndSteeringWheel()
   {
      combinedTerrainObject = createCombinedTerrainObject();
   }

   private CombinedTerrainObject createCombinedTerrainObject()
   {
      CombinedTerrainObject terrainObject = new CombinedTerrainObject("carSeatBox");

      // seat
      terrainObject.addBox(-BOX_LENGTH / 2.0, -BOX_WIDTH / 2.0, BOX_LENGTH / 2.0, BOX_WIDTH / 2.0, BOX_HEIGHT);

      // ground
      terrainObject.addBox(-100.0, -100.0, 100.0, 100.0, -0.05, 0.0, YoAppearance.DarkGray());

      return terrainObject;
   }

   public TerrainObject getTerrainObject()
   {
      return combinedTerrainObject;
   }

   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();    // FIXME
   }

   public void createAndSetContactControllerToARobot()
   {
      // TODO Auto-generated method stub

   }

   public void addContactPoints(ExternalForcePoint[] externalForcePoints)
   {
      // TODO Auto-generated method stub

   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub

   }

   public static void main(String[] args)
   {
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      DRCDemoEnvironmentWithBoxAndSteeringWheel environment = new DRCDemoEnvironmentWithBoxAndSteeringWheel();

      ContactableSelectableSteeringWheelRobot steeringWheel = new ContactableSelectableSteeringWheelRobot("steeringWheel");

      SimulationConstructionSet scs = new SimulationConstructionSet(steeringWheel, 36000);
      scs.setDT(0.001, 1);

      scs.addStaticLinkGraphics(environment.getTerrainObject().getLinkGraphics());

      scs.addDynamicGraphicObjectListRegistries(dynamicGraphicObjectsListRegistry);
//      scs.setGroundVisible(true);

      scs.startOnAThread();
   }

}
