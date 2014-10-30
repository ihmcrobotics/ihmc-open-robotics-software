package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.darpaRoboticsChallenge.ContactController;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.GroundContactModel;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import com.yobotics.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCDebrisEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final List<ContactableSelectableBoxRobot> debrisRobots = new ArrayList<ContactableSelectableBoxRobot>();

   private final String debrisName = "Debris";

   private final double debrisLength = 0.1016;
   private final double debrisWidth = 0.0508;
   private final double debrisHeight = 0.9144;
   private final double debrisMass = 1.0;

   public DRCDebrisEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());

      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      double forceVectorScale = 1.0 / 50.0;
      createBoxes(forceVectorScale, combinedTerrainObject);

   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.DarkBlue());

      return combinedTerrainObject;
   }

//   ContactableSelectableBoxRobot table;
   private void createBoxes(double forceVectorScale, GroundProfile3D groundProfile)
   {
      debrisRobots.add(createDebris(1.0, -0.5, debrisHeight /2.0, 0.0, 0.0, 0.0));
      debrisRobots.add(createDebris(1.0, 0.5, debrisHeight / 2.0, 0.0, 0.0, 0.0));
      debrisRobots.add(createDebris(1.5, -0.5, debrisHeight / 2.0, 0.0, 0.0, 0.0));
      debrisRobots.add(createDebris(1.5, 0.5, debrisHeight / 2.0, 0.0, 0.0, 0.0));

//      table = ContactableSelectableBoxRobot.createContactable2By4Robot("Table", 0.55, 1.05, 0.05, 3.0);
//      table.setPosition(1.25, 0.0, debrisHeight + 0.025);
//      table.setYawPitchRoll(0.0, 0.0, 0.0);
//      debrisRobots.add(table);
    
      for (int i = 0; i < debrisRobots.size(); i++)
      {
         ContactableSelectableBoxRobot debrisRobot = debrisRobots.get(i);
         GroundContactModel groundContactModel = createGroundContactModel(debrisRobot, groundProfile);
         debrisRobot.createAvailableContactPoints(1, 15, forceVectorScale, false);
         debrisRobot.setGroundContactModel(groundContactModel);
      }
   }

   private int id = 0;

   public ContactableSelectableBoxRobot createDebris(double x, double y, double z, double yaw, double pitch, double roll)
   {
      ContactableSelectableBoxRobot debris = ContactableSelectableBoxRobot.createContactable2By4Robot(debrisName + String.valueOf(id++), debrisLength, debrisWidth, debrisHeight, debrisMass);
      debris.setPosition(x, y, z);
      debris.setYawPitchRoll(yaw, pitch, roll);

      return debris;
   }

   private GroundContactModel createGroundContactModel(Robot robot, GroundProfile3D groundProfile)
   {
      double kXY = 5000.0;
      double bXY = 100.0;
      double kZ = 1000.0;
      double bZ = 100.0;
      double alphaStick = 0.7;
      double alphaSlip = 0.5;

      GroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(robot, kXY, bXY, kZ, bZ, alphaSlip, alphaStick,
            robot.getRobotsYoVariableRegistry());
      groundContactModel.setGroundProfile3D(groundProfile);

      return groundContactModel;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<ContactableSelectableBoxRobot> getEnvironmentRobots()
   {
      return debrisRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      // add contact controller to any robot so it gets called
      ContactController contactController = new ContactController();
      contactController.setContactParameters(10000.0, 1000.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(debrisRobots);
      debrisRobots.get(0).setController(contactController);

      // add contact controller to any robot so it gets called
//      ContactController contactController2 = new ContactController("2");
//      contactController2.setContactParameters(10000.0, 1000.0, 0.5, 0.3);
//      contactController2.addContactPoints(table.getAllGroundContactPoints());
//      contactController2.addContactables(debrisRobots.subList(0, debrisRobots.size()-1));
//      table.setController(contactController2);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      this.contactPoints.addAll(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
