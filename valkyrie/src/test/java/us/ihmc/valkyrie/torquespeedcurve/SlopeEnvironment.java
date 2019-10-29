package us.ihmc.valkyrie.torquespeedcurve;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class SlopeEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D slopeGround;
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public SlopeEnvironment(double angle)
   {
      slopeGround = new CombinedTerrainObject3D("slope");

      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");

      RigidBodyTransform location = new RigidBodyTransform();
      location.setTranslation(new Vector3D(0, 0, -0.5));
      location.setRotationPitch(angle);

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, 45, 45, 1), texture);
      slopeGround.addTerrainObject(newBox);
      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3D(location, 200, 200, 0.75), YoAppearance.DarkGray());
      slopeGround.addTerrainObject(newBox2);
   }

   public void addEnvironmentRobot(Robot robot)
   {
      environmentRobots.add(robot);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return slopeGround;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return environmentRobots;
   }
}
