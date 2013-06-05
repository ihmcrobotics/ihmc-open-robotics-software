package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving.VehicleObject;
import us.ihmc.darpaRoboticsChallenge.controllers.SteeringWheelDisturbanceController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.ContactableToroidRobot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObject;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.Contactable;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;

public class DRCDemoEnvironmentWithBoxAndSteeringWheel implements CommonAvatarEnvironmentInterface
{
   private static final double BOX_LENGTH = 0.7;
   private static final double BOX_WIDTH = 3.0;
   private static final double BOX_HEIGHT = 0.85;
   private final CombinedTerrainObject combinedTerrainObject;

   private final ArrayList<Robot> environmentRobots = new ArrayList<Robot>();
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactables = new ArrayList<Contactable>();
   private final ContactableToroidRobot steeringWheelRobot;

   public DRCDemoEnvironmentWithBoxAndSteeringWheel(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DRCVehicleModelObjects drcVehicleModelObjects = new DRCVehicleModelObjects();


      Transform3D steeringWheelTransform = drcVehicleModelObjects.getTransform(VehicleObject.STEERING_WHEEL);


      /*
       * Quick estimates from 3D files:
       */
      double externalSteeringWheelRadius = 0.173;
      double internalSteeringWheelRadius = 0.143;

      double toroidRadius = (externalSteeringWheelRadius - internalSteeringWheelRadius) / 2.0;
      double steeringWheelRadius = (externalSteeringWheelRadius + internalSteeringWheelRadius) / 2.0;

      double xOffsetFromSeat = 0.15;

      combinedTerrainObject = createCombinedTerrainObject(steeringWheelTransform, xOffsetFromSeat);

      double mass = 1.0;
      steeringWheelRobot = new ContactableToroidRobot("steeringWheel", steeringWheelTransform, steeringWheelRadius, toroidRadius, mass);
      steeringWheelRobot.setDamping(2.0);
      steeringWheelRobot.createAvailableContactPoints(1, 30, 0.005, false);
      contactables.add(steeringWheelRobot);
      environmentRobots.add(steeringWheelRobot);
   }

   private CombinedTerrainObject createCombinedTerrainObject(Transform3D steeringWheelTransform, double xOffsetFromSeat)
   {
      CombinedTerrainObject terrainObject = new CombinedTerrainObject("carSeatBox");

      Vector3d steeringWheelTranslation = new Vector3d();
      steeringWheelTransform.get(steeringWheelTranslation);

      // seat
      double xOffset = steeringWheelTranslation.getX() - BOX_LENGTH / 2.0 - xOffsetFromSeat;
      double yOffset = steeringWheelTranslation.getY();
      terrainObject.addBox(-BOX_LENGTH / 2.0 + xOffset, -BOX_WIDTH / 2.0 + yOffset, BOX_LENGTH / 2.0 + xOffset, BOX_WIDTH / 2.0 + yOffset, BOX_HEIGHT);

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
      return new ArrayList<Robot>(environmentRobots);
   }

   public void addContactPoints(ExternalForcePoint[] contactPoints)
   {
      for (ExternalForcePoint contactPoint : contactPoints)
      {
         this.contactPoints.add(contactPoint);
      }
   }

   public void createAndSetContactControllerToARobot()
   {
      // add contact controller to any robot so it gets called
      ContactController contactController = new ContactController();
      contactController.setContactParameters(10000.0, 1000.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(contactables);
      environmentRobots.get(0).setController(contactController);
   }
   
   public void activateDisturbanceControllerOnSteeringWheel(YoFunctionGeneratorMode disturbanceMode)
   {
      SteeringWheelDisturbanceController controller = new SteeringWheelDisturbanceController(steeringWheelRobot, disturbanceMode);
      steeringWheelRobot.setController(controller);
   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      for (Contactable contactable : contactables)
      {
         if (contactable instanceof SelectableObject)
         {
            ((SelectableObject) contactable).addSelectedListeners(selectedListener);
         }
      }
   }
}
