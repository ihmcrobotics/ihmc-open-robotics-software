package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
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
   private static final double BOX_LENGTH = 0.5;
   private static final double BOX_WIDTH = 0.5;
   private static final double BOX_HEIGHT = 0.6;
   private final CombinedTerrainObject combinedTerrainObject;

   private final ArrayList<Robot> environmentRobots = new ArrayList<Robot>();
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactables = new ArrayList<Contactable>();
   private final ContactableToroidRobot steeringWheelRobot;

   public DRCDemoEnvironmentWithBoxAndSteeringWheel(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      /*
       * Quick estimates from 3D files:
       */
      double steeringWheelAngleFromVertical = FastMath.toRadians(50.0);
      double seatEdgeToSteeringWheelCentroidX = 0.18;
      double seatEdgeToSteeringWheelCentroidZ = 0.24;
      double externalSteeringWheelRadius = 0.175;
      double internalSteeringWheelRadius = 0.142;

      double toroidRadius = (externalSteeringWheelRadius - internalSteeringWheelRadius) / 2.0;
      double steeringWheelRadius = (externalSteeringWheelRadius + internalSteeringWheelRadius) / 2.0;
      double steeringWheelAngleFromHorizontal = steeringWheelAngleFromVertical - Math.PI / 2.0;
      double steeringWheelCentroidX = BOX_LENGTH / 2.0 + seatEdgeToSteeringWheelCentroidX;
      double steeringWheelCentroidZ = BOX_HEIGHT + seatEdgeToSteeringWheelCentroidZ;

      combinedTerrainObject = createCombinedTerrainObject();
      Matrix3d pinJointZRotation = new Matrix3d();
      pinJointZRotation.rotZ(-FastMath.PI / 2.0);
      Matrix3d pinJointRotation = new Matrix3d();
      pinJointRotation.rotY(steeringWheelAngleFromHorizontal);
      pinJointRotation.mul(pinJointZRotation);

      Vector3d pinJointLocation = new Vector3d(steeringWheelCentroidX, 0.0, steeringWheelCentroidZ);
      Transform3D pinJointTransform = new Transform3D(pinJointRotation, pinJointLocation, 1.0);

      double mass = 1.0;
      steeringWheelRobot = new ContactableToroidRobot("steeringWheel", pinJointTransform, steeringWheelRadius, toroidRadius, mass);
      steeringWheelRobot.setDamping(2.0);
      steeringWheelRobot.createAvailableContactPoints(1, 30, 0.005, false);
      contactables.add(steeringWheelRobot);
      environmentRobots.add(steeringWheelRobot);
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
