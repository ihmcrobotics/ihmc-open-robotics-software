package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class FlatGroundWithRubbleEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D terrain;
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public FlatGroundWithRubbleEnvironment()
   {
      terrain = DefaultCommonAvatarEnvironment.setUpGround("Ground");

      Random random = new Random(413);

      Box3D twoByFour = new Box3D(4.0 * 0.0254, 1.0, 2.0 * 0.0254);

      double lastX = 0.5;

      for (int i = 0; i < 10; i++)
      {
         twoByFour.getPose().setToZero();
         twoByFour.getPose().getTranslation().set(lastX, EuclidCoreRandomTools.nextDouble(random, 0.6), 0.0);
         twoByFour.getPose().prependTranslation(EuclidCoreRandomTools.nextDouble(random, 0.0, 0.50), 0.0, 0.0);
         twoByFour.getPose().getRotation().setToYawOrientation(EuclidCoreRandomTools.nextDouble(random, Math.PI / 4.0));
//         twoByFour.getPose().getRotation().appendRollRotation(EuclidCoreRandomTools.nextDouble(random, Math.toRadians(5.0)));
         addBox3DAndAdjustHeight(twoByFour, new YoAppearanceTexture("Textures/brick.png"));

         lastX = twoByFour.getPose().getTranslationX();
      }
   }

   private void addBox3DAndAdjustHeight(Box3DReadOnly box, AppearanceDefinition appearance)
   {
      Box3D boxCopy = new Box3D(box);
      Point3D supportingVertex = new Point3D(boxCopy.getSupportingVertex(Axis3D.Z.negated()));
      double heightAt = this.getTerrainObject3D().getHeightMapIfAvailable().heightAt(supportingVertex.getX(), supportingVertex.getY(), 10.0);
      boxCopy.getPose().getTranslation().addZ(heightAt - supportingVertex.getZ());
      terrain.addRotatableBox(boxCopy, appearance);
   }

   public void addTwoByFour(Point3DReadOnly position, Orientation3DReadOnly orientation, double length, AppearanceDefinition appearance)
   {
      addTwoByFour(new Pose3D(position, orientation), length, appearance);
   }

   public void addTwoByFour(Pose3DReadOnly pose, double length, AppearanceDefinition appearance)
   {
      Box3D box = new Box3D(2.0 * 0.0254, 4.0 * 0.025, length);
      box.getPose().set(pose);
      terrain.addRotatableBox(box, appearance);
   }

   public void addEnvironmentRobot(Robot robot)
   {
      environmentRobots.add(robot);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return terrain;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return environmentRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
