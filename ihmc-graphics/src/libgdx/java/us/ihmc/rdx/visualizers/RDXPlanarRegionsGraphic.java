package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.mesh.RDXIDMappedColorFunction;
import us.ihmc.rdx.mesh.RDXMeshGraphicTools;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.function.Function;

public class RDXPlanarRegionsGraphic implements RenderableProvider
{
   private PlanarRegionsList planarRegionsList;
   private final ModelBuilder modelBuilder = new ModelBuilder();

   // visualization options
   private Function<Integer, Color> colorFunction = new RDXIDMappedColorFunction();
   private float opacity = 1.0f;
   private boolean drawAreaText = false;
   private boolean drawBoundingBox = false;
   private boolean drawNormal;
   boolean mouseHovering = false;

   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private String tooltipText = "";
   private RDX3DPanelTooltip tooltip;
   private RDXModelInstance modelInstance;
   private Model lastModel;
   private Texture paletteTexture = null;

   private int selectedRegionId = -1;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void clear()
   {
      generateMeshes(new PlanarRegionsList());
   }

   public void generateFlatGround()
   {
      generateMeshes(PlanarRegionsList.flatGround(20.0));
   }

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void generateMeshesAsync(PlanarRegionsList planarRegionsList)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(planarRegionsList));
   }

   public synchronized void generateMeshes(PlanarRegionsList incomingPlanarRegionsList)
   {
      planarRegionsList = incomingPlanarRegionsList;
      // if we're passing in null, make an empty list
      if (incomingPlanarRegionsList == null)
         incomingPlanarRegionsList = new PlanarRegionsList();

      ArrayList<RDXMultiColorMeshBuilder> meshBuilders = new ArrayList<>();
      for (PlanarRegion planarRegion : incomingPlanarRegionsList.getPlanarRegionsAsList())
      {
         RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
         meshBuilders.add(meshBuilder);
         singleRegionMeshBuilder(planarRegion, meshBuilder); // should do in parallel somehow?
      }
      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Material material = new Material();
         if (paletteTexture == null)
            paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
         material.set(PBRTextureAttribute.createBaseColorTexture(paletteTexture));
         material.set(PBRColorAttribute.createBaseColorFactor(new Color(0.7f, 0.7f, 0.7f, 1.0f)));

         for (RDXMultiColorMeshBuilder meshBuilder : meshBuilders)
         {
            Mesh mesh = meshBuilder.generateMesh();
            MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
            modelBuilder.part(meshPart, material);
         }

         if (lastModel != null)
            lastModel.dispose();

         lastModel = modelBuilder.end();
         modelInstance = new RDXModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
         modelInstance.setOpacity(opacity);
      };
   }

   private void singleRegionMeshBuilder(PlanarRegion planarRegion, RDXMultiColorMeshBuilder meshBuilder)
   {
      RigidBodyTransform transformToWorld = planarRegion.getTransformToWorldCopy();
      Color color = colorFunction.apply(planarRegion.getRegionId());

      meshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull(), 0.01, color, true);

      double totalArea = 0.0;
      for (ConvexPolygon2D convexPolygon : planarRegion.getConvexPolygons())
      {
         meshBuilder.addPolygon(transformToWorld, convexPolygon, color);

         totalArea += convexPolygon.getArea();
      }

//      LabelGraphic sizeLabel = null;
//      if (drawAreaText)
//      {
//         sizeLabel = new LabelGraphic(FormattingTools.getFormattedToSignificantFigures(totalArea, 3));
//         sizeLabel.getPose().appendTransform(transformToWorld);
//         sizeLabel.update();
//      }

      if (drawBoundingBox)
      {
//         RDXMeshGraphicTools.drawBoxEdges(meshBuilder, PlanarRegionTools.getLocalBoundingBox3DInWorld(planarRegion, 0.1), 0.005, color);

         BoundingBox3D boundingBox3D = planarRegion.getBoundingBox3dInWorld();
         Box3D box = GeometryTools.convertBoundingBox3DToBox3D(boundingBox3D);
         RDXMeshGraphicTools.drawBoxEdges(meshBuilder, box, 0.005, color);
      }

      if (drawNormal)
      {
         Point3DReadOnly centroid = PlanarRegionTools.getCentroid3DInWorld(planarRegion);

         double length = 0.07;
         double radius = 0.004;
         double cylinderToConeLengthRatio = 0.8;
         double coneDiameterMultiplier = 1.8;
         RDXMeshGraphicTools.drawArrow(meshBuilder,
                                       centroid,
                                       transformToWorld.getRotation(),
                                       length,
                                       radius,
                                       cylinderToConeLengthRatio,
                                       coneDiameterMultiplier, color);
      }


   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      Line3DReadOnly pickRayInWorld = input.getPickRayInWorld();

      if (pickRayInWorld != null)
      {
         if (planarRegionsList != null)
         {
            ImmutablePair<Point3D, PlanarRegion> regionsWithRay = PlanarRegionTools.intersectRegionsWithRay(planarRegionsList,
                                                                                                            pickRayInWorld.getPoint(),
                                                                                                            new Vector3D(pickRayInWorld.getDirection()));

            if (regionsWithRay != null)
            {
               mouseHovering = true;
               selectedRegionId = regionsWithRay.getRight().getRegionId();

               tooltipText = regionsWithRay.getRight().getDebugString();
               return;
            }

         }
      }

      selectedRegionId = -1;
      mouseHovering = false;
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
//      modelInstance.setOpacity(mouseHovering ? 0.5f : 1.0f);
      if (tooltip != null)
         tooltip.setInput(input);
   }

   public void setupTooltip(RDX3DPanel panel3D, String text)
   {
      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(() ->
                                      {
                                         if (mouseHovering)
                                            tooltip.render(tooltipText);
                                      });
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

   public void setDrawAreaText(boolean drawAreaText)
   {
      this.drawAreaText = drawAreaText;
   }

   public void setDrawBoundingBox(boolean drawBoundingBox)
   {
      this.drawBoundingBox = drawBoundingBox;
   }

   public void setDrawNormal(boolean drawNormal)
   {
      this.drawNormal = drawNormal;
   }

   public void setColorFunction(Function<Integer, Color> colorFunction)
   {
      this.colorFunction = colorFunction;
   }

   public void setBlendOpacity(float opacity)
   {
      this.opacity = opacity;

      if (modelInstance != null)
         modelInstance.setOpacity(opacity);
   }
}
