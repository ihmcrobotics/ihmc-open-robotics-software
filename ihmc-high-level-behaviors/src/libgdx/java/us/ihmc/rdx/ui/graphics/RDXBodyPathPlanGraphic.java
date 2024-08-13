package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PathTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.List;

public class RDXBodyPathPlanGraphic implements RenderableProvider
{
   private static final double LINE_THICKNESS = 0.025;
   private final float startColorHue;
   private final float goalColorHue;
   {
      float[] hsv = new float[3];
      Color.GREEN.toHsv(hsv);
      startColorHue = hsv[0];
      Color.RED.toHsv(hsv);
      goalColorHue = hsv[0];
   }

   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private final ModelBuilder modelBuilder = new ModelBuilder();
   private ModelInstance modelInstance;
   private Model lastModel;
   private Texture paletteTexture = null;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void clear()
   {
      generateMeshes(new ArrayList<>());
   }

   /**
    * To process in parallel.
    */
   public void generateMeshesAsync(List<? extends Pose3DReadOnly> bodyPath)
   {
      executorService.submit(() -> generateMeshes(bodyPath));
   }

   public void generateMeshes(List<? extends Pose3DReadOnly> bodyPath)
   {
      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();

      double totalPathLength = PathTools.computePosePathLength(bodyPath);
      double currentLength = 0.0;

      for (int segmentIndex = 0; segmentIndex < bodyPath.size() - 1; segmentIndex++)
      {
         Point3DReadOnly lineStart = bodyPath.get(segmentIndex).getPosition();
         Point3DReadOnly lineEnd = bodyPath.get(segmentIndex + 1).getPosition();

         float lineStartHue = (float) EuclidCoreTools.interpolate(startColorHue, goalColorHue, currentLength / totalPathLength);
         currentLength += lineStart.distance(lineEnd);
         float lineEndHue = (float) EuclidCoreTools.interpolate(startColorHue, goalColorHue, currentLength / totalPathLength);
         meshBuilder.addLine(lineStart, lineEnd, LINE_THICKNESS, new Color().fromHsv(lineStartHue, 1.0f, 0.5f), new Color().fromHsv(lineEndHue, 1.0f, 1.0f));
      }
      for (int waypointIndex = 0; waypointIndex < bodyPath.size(); waypointIndex++)
      {
         createCoordinateFrame(bodyPath.get(waypointIndex), meshBuilder, 0.2);
      }

      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         com.badlogic.gdx.graphics.g3d.Material material = new Material();
         if (paletteTexture == null)
            paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
         material.set(PBRTextureAttribute.createBaseColorTexture(paletteTexture));
         material.set(PBRColorAttribute.createBaseColorFactor(new Color(0.7f, 0.7f, 0.7f, 1.0f)));
         modelBuilder.part(meshPart, material);
         if (lastModel != null)
            lastModel.dispose();

         lastModel = modelBuilder.end();
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };
   }

   private static void createCoordinateFrame(Pose3DReadOnly pose, RDXMultiColorMeshBuilder meshBuilder, double length)
   {
      double radius = 0.02 * length;
      double coneHeight = 0.10 * length;
      double coneRadius = 0.05 * length;

      Point3D cylinderPoint = new Point3D();
      AxisAngle xOrientation = new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0);
      AxisAngle yOrientation = new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0);
      AxisAngle zOrientation = new AxisAngle();

      Point3D xConePoint = new Point3D(length, 0.0, 0.0);
      Point3D yConePoint = new Point3D(0.0, length, 0.0);
      Point3D zConePoint = new Point3D(0.0, 0.0, length);


      pose.transform(cylinderPoint);
      pose.transform(xOrientation);
      pose.transform(yOrientation);
      pose.transform(zOrientation);
      pose.transform(xConePoint);
      pose.transform(yConePoint);
      pose.transform(zConePoint);

      meshBuilder.addCylinder(length, radius, cylinderPoint, xOrientation, Color.RED);
      meshBuilder.addCone(coneHeight, coneRadius, xConePoint, xOrientation, Color.RED);
      meshBuilder.addCylinder(length, radius, cylinderPoint, yOrientation, Color.GREEN);
      meshBuilder.addCone(coneHeight, coneRadius, yConePoint, yOrientation, Color.GREEN);
      meshBuilder.addCylinder(length, radius, cylinderPoint, zOrientation, Color.BLUE);
      meshBuilder.addCone(coneHeight, coneRadius, zConePoint, zOrientation, Color.BLUE);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // sync over current and add
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      executorService.destroy();
   }
}
