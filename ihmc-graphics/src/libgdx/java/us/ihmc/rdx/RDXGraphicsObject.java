package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.badlogic.gdx.utils.SerializationException;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.*;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DRotateInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DTranslateInstruction;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class RDXGraphicsObject extends Graphics3DInstructionExecutor implements RenderableProvider
{
   private final ArrayList<Model> models = new ArrayList<>();
   private final ArrayList<ModelInstance> modelInstances = new ArrayList<>();
   private final HashMap<ModelInstance, AffineTransform> transforms = new HashMap<>();
   private final AffineTransform placementTransform = new AffineTransform();
   private final AffineTransform tempTransform = new AffineTransform();

   public RDXGraphicsObject(Graphics3DObject graphics3dObject)
   {
      this(graphics3dObject, null);
   }

   public RDXGraphicsObject(Graphics3DObject graphics3dObject, AppearanceDefinition appearance)
   {
      if (graphics3dObject != null)
      {
         List<Graphics3DPrimitiveInstruction> graphics3dInstructions = graphics3dObject.getGraphics3DInstructions();
         if (graphics3dInstructions != null)
         {
            for (Graphics3DPrimitiveInstruction instruction : graphics3dInstructions)
            {
               if (instruction instanceof Graphics3DInstruction)
               {
                  Graphics3DInstruction graphicsInstruction = (Graphics3DInstruction) instruction;
                  if (appearance != null)
                     graphicsInstruction.setAppearance(appearance);
               }
            }
            setUpGraphicsFromDefinition(graphics3dInstructions);
         }
      }
   }

   @Override
   protected void doAddMeshDataInstruction(Graphics3DAddMeshDataInstruction graphics3DAddMeshData)
   {
//      graphics3DAddMeshData.getMeshData().getVertices();
//      TriangleMesh outputMesh = interpretMeshData(graphics3DAddMeshData.getMeshData());
//      Material outputMaterial = convertMaterial(graphics3DAddMeshData.getAppearance());
//
//      MeshView meshView = new MeshView();
//      meshView.setMesh(outputMesh);
//      meshView.setMaterial(outputMaterial);
//      Node meshGroup = new Node();
//      meshGroup.
//      currentNode.addChild(meshGroup);
//      currentNode = meshGroup;
   }

   @Override
   protected void doAddHeightMapInstruction(Graphics3DAddHeightMapInstruction graphics3DAddHeightMap)
   {
      // not implemented yet
   }

   @Override
   protected void doAddExtrusionInstruction(Graphics3DAddExtrusionInstruction graphics3DAddText)
   {
      // not implemented yet
   }

   @Override
   protected void doAddModelFileInstruction(Graphics3DAddModelFileInstruction graphics3DAddModelFile)
   {
      try
      {
         String modelFileName = graphics3DAddModelFile.getFileName();

         if (modelFileName == null)
            return;

         Model model = RDXModelLoader.load(modelFileName);
         if (model == null)
         {
            LogTools.warn("Could not load {}", modelFileName);
            return;
         }
         models.add(model);

         if (graphics3DAddModelFile.getAppearance() != null)
         {
            AppearanceDefinition appearance = graphics3DAddModelFile.getAppearance();
            Color color = LibGDXTools.toLibGDX(appearance);

            if (model.materials.size == 0)
            {
               model.materials.add(new Material());
            }

            Material firstMaterial = model.materials.get(0);

            if (firstMaterial != null)
            {
               firstMaterial.set(ColorAttribute.createDiffuse(color.r, color.g, color.b, color.a));
               if (color.a < 1.0f)
               {
                  firstMaterial.set(new BlendingAttribute(true, color.a));
               }
            }
         }

         ModelInstance modelInstance = new ModelInstance(model);
         transforms.put(modelInstance, new AffineTransform(placementTransform));
         modelInstances.add(modelInstance);
      }
      catch (SerializationException e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }

   }

   @Override
   protected void doIdentityInstruction()
   {
      placementTransform.setIdentity();
   }

   @Override
   protected void doRotateInstruction(Graphics3DRotateInstruction rotateInstruction)
   {
      placementTransform.appendOrientation(rotateInstruction.getRotationMatrix());
   }

   @Override
   protected void doScaleInstruction(Graphics3DScaleInstruction graphics3DScale)
   {
      placementTransform.appendScale(graphics3DScale.getScaleFactor());
   }

   @Override
   protected void doTranslateInstruction(Graphics3DTranslateInstruction graphics3DTranslate)
   {
      placementTransform.appendTranslation(graphics3DTranslate.getTranslation());
   }

   @Override
   protected void doAddPrimitiveInstruction(PrimitiveGraphics3DInstruction primitiveInstruction)
   {
      if (primitiveInstruction instanceof CubeGraphics3DInstruction)
      {
         CubeGraphics3DInstruction cubeInstruction = (CubeGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Cube(cubeInstruction.getLength(),
                                                          cubeInstruction.getWidth(),
                                                          cubeInstruction.getHeight(),
                                                          cubeInstruction.getCenteredInTheCenter(),
                                                          cubeInstruction.getTextureFaces());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, cubeInstruction.getAppearance());

         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof SphereGraphics3DInstruction)
      {
         SphereGraphics3DInstruction sphereInstruction = (SphereGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Sphere(sphereInstruction.getRadius(),
                                                            sphereInstruction.getResolution(),
                                                            sphereInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, sphereInstruction.getAppearance());

         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof WedgeGraphics3DInstruction)
      {
         WedgeGraphics3DInstruction wedgeInstruction = (WedgeGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Wedge(wedgeInstruction.getLengthX(), wedgeInstruction.getWidthY(), wedgeInstruction.getHeightZ());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, wedgeInstruction.getAppearance());

         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof CapsuleGraphics3DInstruction)
      {
         CapsuleGraphics3DInstruction capsuleInstruction = (CapsuleGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Capsule(capsuleInstruction.getHeight(),
                                                             capsuleInstruction.getXRadius(),
                                                             capsuleInstruction.getYRadius(),
                                                             capsuleInstruction.getZRadius(),
                                                             capsuleInstruction.getResolution(),
                                                             capsuleInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, capsuleInstruction.getAppearance());

         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof EllipsoidGraphics3DInstruction)
      {
         EllipsoidGraphics3DInstruction ellipsoidInstruction = (EllipsoidGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Ellipsoid(ellipsoidInstruction.getXRadius(),
                                                               ellipsoidInstruction.getYRadius(),
                                                               ellipsoidInstruction.getZRadius(),
                                                               ellipsoidInstruction.getResolution(),
                                                               ellipsoidInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, ellipsoidInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof CylinderGraphics3DInstruction)
      {
         CylinderGraphics3DInstruction cylinderInstruction = (CylinderGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Cylinder(cylinderInstruction.getRadius(),
                                                              cylinderInstruction.getHeight(),
                                                              cylinderInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, cylinderInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof ConeGraphics3DInstruction)
      {
         ConeGraphics3DInstruction coneInstruction = (ConeGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Cone(coneInstruction.getHeight(), coneInstruction.getRadius(), coneInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, coneInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof TruncatedConeGraphics3DInstruction)
      {
         TruncatedConeGraphics3DInstruction truncatedConeInstruction = (TruncatedConeGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.GenTruncatedCone(truncatedConeInstruction.getHeight(),
                                                                      truncatedConeInstruction.getXBaseRadius(),
                                                                      truncatedConeInstruction.getYBaseRadius(),
                                                                      truncatedConeInstruction.getXTopRadius(),
                                                                      truncatedConeInstruction.getYTopRadius(),
                                                                      truncatedConeInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, truncatedConeInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof HemiEllipsoidGraphics3DInstruction)
      {
         HemiEllipsoidGraphics3DInstruction hemiEllipsoidInstruction = (HemiEllipsoidGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.HemiEllipsoid(hemiEllipsoidInstruction.getXRadius(),
                                                                   hemiEllipsoidInstruction.getYRadius(),
                                                                   hemiEllipsoidInstruction.getZRadius(),
                                                                   hemiEllipsoidInstruction.getResolution(),
                                                                   hemiEllipsoidInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, hemiEllipsoidInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof ArcTorusGraphics3DInstruction)
      {
         ArcTorusGraphics3DInstruction arcTorusInstruction = (ArcTorusGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.ArcTorus(arcTorusInstruction.getStartAngle(),
                                                              arcTorusInstruction.getEndAngle(),
                                                              arcTorusInstruction.getMajorRadius(),
                                                              arcTorusInstruction.getMinorRadius(),
                                                              arcTorusInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, arcTorusInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof PyramidCubeGraphics3DInstruction)
      {
         PyramidCubeGraphics3DInstruction pyramidInstruction = (PyramidCubeGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.PyramidCube(pyramidInstruction.getLengthX(),
                                                                 pyramidInstruction.getWidthY(),
                                                                 pyramidInstruction.getHeightZ(),
                                                                 pyramidInstruction.getPyramidHeight());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, pyramidInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof PolygonGraphics3DInstruction)
      {
         PolygonGraphics3DInstruction polygonInstruction = (PolygonGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Polygon(polygonInstruction.getPolygonPoints());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, polygonInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof ExtrudedPolygonGraphics3DInstruction)
      {
         ExtrudedPolygonGraphics3DInstruction extrudedPolygonInstruction = (ExtrudedPolygonGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.ExtrudedPolygon(extrudedPolygonInstruction.getPolygonPoints(),
                                                                     extrudedPolygonInstruction.getExtrusionHeight());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData,
                                                                                                           extrudedPolygonInstruction.getAppearance());
         doAddMeshDataInstruction(meshDataInstruction);
      }
      else
      {
         throw new RuntimeException("Need to support that primitive type! primitiveInstruction = " + primitiveInstruction);
      }
   }

   public void setWorldTransform(AffineTransform worldTransform)
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         tempTransform.set(transforms.get(modelInstance));
         worldTransform.transform(tempTransform);
         LibGDXTools.toLibGDX(tempTransform, modelInstance.transform);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      for (Model model : models)
      {
         model.dispose();
      }
   }
}
