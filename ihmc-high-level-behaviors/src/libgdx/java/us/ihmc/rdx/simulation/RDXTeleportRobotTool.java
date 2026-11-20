package us.ihmc.rdx.simulation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Hold {@link #KEY} over the 3D view to show where the robot's mid-feet point will go: the highest point of the ground truth
 * terrain under the mouse. The pointer in front of the disc is the pelvis facing, starting at the robot's current pelvis yaw; while holding, the left and right arrow
 * keys rotate it counterclockwise and clockwise. Releasing calls the teleport callback with that pose, e.g. to reset a simulation there.
 */
public class RDXTeleportRobotTool
{
   public static final char KEY = 'T';
   private static final double RADIUS = 0.3;
   private static final double THICKNESS = 0.01;
   // Same proportions as the RDXPathControlRingGizmo pointers, scaled from its 0.426 m ring to this disc
   private static final double POINTER_SCALE = RADIUS / 0.426;
   private static final double POINTER_WIDTH = 0.257 * POINTER_SCALE;
   private static final double POINTER_HEIGHT = 0.137 * POINTER_SCALE;
   private static final double POINTER_SPACING = 0.079 * POINTER_SCALE;
   private static final double YAW_RATE = Math.toRadians(90.0);
   private static final int PROJECTION_ITERATIONS = 6;

   private final Point3D goal = new Point3D();
   private final RigidBodyTransform goalPose = new RigidBodyTransform();
   private double yaw;
   private DoubleSupplier initialYaw;
   private RDXModelInstance goalGraphic;
   private boolean active = false;
   private boolean goalValid = false;

   /**
    * @param groundTruthTerrain the terrain the simulation collides with, or null for flat ground at z = 0
    * @param initialYaw         the robot's current pelvis yaw, read each time the key is pressed, where the pointer starts
    * @param teleport           called with the mid-feet goal pose, position and yaw, when the key is released
    */
   public void create(RDXBaseUI baseUI,
                      Supplier<? extends TerrainObject3D> groundTruthTerrain,
                      DoubleSupplier initialYaw,
                      Consumer<Pose3DReadOnly> teleport)
   {
      this.initialYaw = initialYaw;
      RDXBaseUI.getInstance().getKeyBindings().register("Teleport robot: hold, release to reset the sim there", String.valueOf(KEY));
      RDXBaseUI.getInstance().getKeyBindings().register("Teleport robot: rotate counterclockwise / clockwise", "T + Left / Right arrow");
      Color color = new Color(0.1f, 0.6f, 1.0f, 1.0f);
      goalGraphic = new RDXModelInstance(RDXModelBuilder.buildModel(meshBuilder ->
      {
         meshBuilder.addCylinder(THICKNESS, RADIUS, new Point3D(), color);
         // Flat triangle in front of the disc along +X, the facing direction, like the control ring's +X pointer
         meshBuilder.addIsoscelesTriangularPrism(POINTER_WIDTH,
                                                 POINTER_HEIGHT,
                                                 THICKNESS,
                                                 new Point3D(RADIUS + POINTER_SPACING, 0.0, THICKNESS / 2.0),
                                                 new YawPitchRoll(-Math.PI / 2.0, 0.0, -Math.PI / 2.0),
                                                 color);
      }, "TeleportGoal"));
      goalGraphic.setOpacity(0.6f);

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> processInput(input, groundTruthTerrain, teleport));
      // Not GROUND_TRUTH, so simulated cameras do not see it
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables, RDXSceneLevel.VIRTUAL);
   }

   private void processInput(ImGui3DViewInput input, Supplier<? extends TerrainObject3D> groundTruthTerrain, Consumer<Pose3DReadOnly> teleport)
   {
      boolean typing = ImGui.getIO().getWantTextInput();
      if (!active && input.isWindowHovered() && !typing && ImGui.isKeyDown(KEY))
      {
         active = true;
         goalValid = false;
         yaw = initialYaw.getAsDouble();
      }
      if (!active)
         return;

      double yawStep = YAW_RATE * ImGui.getIO().getDeltaTime();
      if (ImGui.isKeyDown(ImGuiTools.getLeftArrowKey()))
         yaw += yawStep;
      if (ImGui.isKeyDown(ImGuiTools.getRightArrowKey()))
         yaw -= yawStep;

      if (input.isWindowHovered() && projectOnTerrain(input.getPickRayInWorld(), groundTruthTerrain == null ? null : groundTruthTerrain.get()))
      {
         goalValid = true;
      }
      if (goalValid)
      {
         goalPose.getRotation().setToYawOrientation(yaw);
         goalPose.getTranslation().set(goal.getX(), goal.getY(), goal.getZ() + 0.005);
         goalGraphic.setPoseInWorldFrame(goalPose);
      }

      if (!ImGui.isKeyDown(KEY))
      {
         active = false;
         if (goalValid)
            teleport.accept(new Pose3D(goal, new Quaternion(yaw, 0.0, 0.0)));
      }
   }

   /**
    * Intersects the pick ray with the horizontal plane at the terrain height found so far, and takes the highest terrain at that
    * x, y as the next height, until it settles.
    */
   private boolean projectOnTerrain(Line3DReadOnly pickRay, TerrainObject3D terrain)
   {
      double directionZ = pickRay.getDirection().getZ();
      if (directionZ > -1.0e-6)
         return false; // Not looking down

      double z = goalValid ? goal.getZ() : 0.0;
      double x = 0.0;
      double y = 0.0;
      for (int i = 0; i < PROJECTION_ITERATIONS; i++)
      {
         double t = (z - pickRay.getPoint().getZ()) / directionZ;
         x = pickRay.getPoint().getX() + t * pickRay.getDirection().getX();
         y = pickRay.getPoint().getY() + t * pickRay.getDirection().getY();
         if (terrain == null)
            break;
         double height = highestTerrainHeight(terrain, x, y);
         if (!Double.isFinite(height) || Math.abs(height - z) < 1.0e-4)
         {
            if (Double.isFinite(height))
               z = height;
            break;
         }
         z = height;
      }
      goal.set(x, y, z);
      return true;
   }

   /**
    * @return the highest terrain surface at x, y, or -infinity if there is none. {@link HeightMap#heightAt} only considers objects
    *       whose bounding box contains the query z, so each object is queried at the top of its own bounding box.
    */
   public static double highestTerrainHeight(TerrainObject3D terrain, double x, double y)
   {
      if (terrain instanceof CombinedTerrainObject3D combined)
      {
         double highest = Double.NEGATIVE_INFINITY;
         for (TerrainObject3D child : combined.getTerrainObjects())
            highest = Math.max(highest, highestTerrainHeight(child, x, y));
         return highest;
      }
      HeightMap heightMap = terrain.getHeightMapIfAvailable();
      BoundingBox3DReadOnly boundingBox = terrain.getBoundingBox();
      if (heightMap == null || boundingBox == null)
         return Double.NEGATIVE_INFINITY;
      if (x < boundingBox.getMinX() || x > boundingBox.getMaxX() || y < boundingBox.getMinY() || y > boundingBox.getMaxY())
         return Double.NEGATIVE_INFINITY;
      return heightMap.heightAt(x, y, boundingBox.getMaxZ());
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (active && goalValid)
      {
         goalGraphic.getRenderables(renderables, pool);
      }
   }
}
