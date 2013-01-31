package us.ihmc.darpaRoboticsChallenge;

import java.util.List;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;

public class DRCDemoEnvironmentWithBoxAndSteeringWheel implements CommonAvatarEnvironmentInterface
{
   private static final float BOX_DIMENSION = 0.5f;
   private final CombinedTerrainObject combinedTerrainObject;

   public DRCDemoEnvironmentWithBoxAndSteeringWheel()
   {
      combinedTerrainObject = createCarSeatBox();

   }


   private CombinedTerrainObject createCarSeatBox()
   {
      CombinedTerrainObject terrainObject = new CombinedTerrainObject("carSeatBox");
      terrainObject.addBox(0, 0, BOX_DIMENSION, BOX_DIMENSION, BOX_DIMENSION);

      return terrainObject;
   }

   public TerrainObject getTerrainObject()
   {
      return combinedTerrainObject;
   }

   public List<Robot> getEnvironmentRobots()
   {
      // TODO Auto-generated method stub
      return null;
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
      DRCDemoEnvironmentWithBoxAndSteeringWheel env = new DRCDemoEnvironmentWithBoxAndSteeringWheel();

      ContactableSelectableSteeringWheelRobot bot = new ContactableSelectableSteeringWheelRobot("steeringWheel");

      SimulationConstructionSet scs = new SimulationConstructionSet(bot, 36000);
      scs.setDT(0.001, 1);

      scs.addStaticLinkGraphics(env.getTerrainObject().getLinkGraphics());

      scs.addDynamicGraphicObjectListRegistries(dynamicGraphicObjectsListRegistry);
      scs.setGroundVisible(true);

      scs.startOnAThread();
   }

}
