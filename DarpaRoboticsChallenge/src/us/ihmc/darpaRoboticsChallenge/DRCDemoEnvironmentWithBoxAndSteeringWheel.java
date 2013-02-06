package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObject;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.Contactable;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;

public class DRCDemoEnvironmentWithBoxAndSteeringWheel implements CommonAvatarEnvironmentInterface
{
   private static final double BOX_LENGTH = 0.5;
   private static final double BOX_WIDTH = 0.5;
   private static final double BOX_HEIGHT = 0.6;
   private final CombinedTerrainObject combinedTerrainObject;

   private final ArrayList<Robot> boxRobots = new ArrayList<Robot>();
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactables = new ArrayList<Contactable>();

   public DRCDemoEnvironmentWithBoxAndSteeringWheel()
   {
      combinedTerrainObject = createCombinedTerrainObject();
      Matrix3d pinJointZRotation = new Matrix3d();
      pinJointZRotation.rotZ(-FastMath.PI / 2.0);
      Matrix3d pinJointRotation = new Matrix3d();
      pinJointRotation.rotY(-FastMath.PI / 4.0);
      pinJointRotation.mul(pinJointZRotation);

      Vector3d pinJointLocation = new Vector3d(0.6, 0.0, 0.9);
      Transform3D pinJointTransformFromWorld = new Transform3D(pinJointRotation, pinJointLocation, 1.0);
      Vector3d pinJointLinkCoMOffset = new Vector3d(0.0, 0.0, 0.05);

      ContactableSteeringWheelRobot bot = new ContactableSteeringWheelRobot("steeringWheel", pinJointTransformFromWorld, pinJointLinkCoMOffset);
      bot.createAvailableContactPoints(1, 30, 1.0 / 2.0);
      contactables.add(bot);
      boxRobots.add(bot);
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
      return new ArrayList<Robot>(boxRobots);
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
      boxRobots.get(0).setController(contactController);
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
