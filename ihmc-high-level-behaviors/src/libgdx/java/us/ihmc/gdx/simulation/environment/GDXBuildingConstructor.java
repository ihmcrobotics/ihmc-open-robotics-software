package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.object.GDXSimpleObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXBuildingObject;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.gizmo.StepCheckIsPointInsideAlgorithm;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class GDXBuildingConstructor extends ImGuiPanel
{
   private enum Mode
   {
      NONE, CONSTRUCTING, PLACING, DONE
   }

   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironmentBuilder.class, "Constructor");
   private final ArrayList<GDXSimpleObject> virtualObjects = new ArrayList<>();
   private GDXSimpleObject selectedObject;
   private GDXSimpleObject intersectedObject;
   private final ImFloat ambientLightAmount = new ImFloat(0.4f);
   private final GDXPose3DGizmo pose3DGizmo = new GDXPose3DGizmo();
   private final ImGuiPanel poseGizmoTunerPanel = pose3DGizmo.createTunerPanel(getClass().getSimpleName());

   private final StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();
   private final GDX3DSceneManager sceneManager;
   private final Point3D tempIntersection = new Point3D();

   private final RigidBodyTransform translationTransform = new RigidBodyTransform();

   private Point3D lastPickPoint = new Point3D();

   private Point3D cornerPoint;

   private final ColorAttribute highlightColor = ColorAttribute.createDiffuse(0.8f, 0.6f, 0.2f, 1.0f);

   private GDXBuildingObject building;
   private GDXSimpleObject lastWallBase;
   private Mode mode = Mode.NONE;

   public GDXBuildingConstructor(GDX3DSceneManager sceneManager)
   {
      super(WINDOW_NAME);
      this.sceneManager = sceneManager;
      setRenderMethod(this::renderImGuiWidgets);
      addChild(poseGizmoTunerPanel);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      sceneManager.addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
      pose3DGizmo.create(sceneManager.getCamera3D());
      baseUI.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXSimpleObject model : virtualObjects)
      {
         model.getRealRenderables(renderables, pool);
      }
      if (selectedObject != null)
      {
         pose3DGizmo.getRenderables(renderables, pool);
      }
      if (intersectedObject != null && intersectedObject != selectedObject)
      {
         intersectedObject.getCollisionMeshRenderables(renderables, pool);
      }
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if(building != null)
      {
         for (GDXSimpleObject model : building.getAllObjects())
         {
            model.getRealRenderables(renderables, pool);
         }
      }
      if (selectedObject != null)
      {
         pose3DGizmo.getRenderables(renderables, pool);
         selectedObject.getCollisionMeshRenderables(renderables, pool);
      }
      if (intersectedObject != null && intersectedObject != selectedObject)
      {
         intersectedObject.getCollisionMeshRenderables(renderables, pool);
      }
   }

   public void process3DViewInput(ImGui3DViewInput viewInput)
   {
      constructionUpdate(viewInput);
      if (selectedObject != null)
      {
         if (mode == Mode.PLACING)
         {
            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
            {
               mode = Mode.DONE;
            }
            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               building.addCorner(cornerPoint);
               mode = Mode.CONSTRUCTING;
            }
         }
         else
         {
            pose3DGizmo.process3DViewInput(viewInput);
            selectedObject.setTransformToWorld(pose3DGizmo.getTransformToParent());

            intersectedObject = calculatePickedObject(viewInput.getPickRayInWorld());
            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               if (intersectedObject != selectedObject)
               {
                  updateObjectSelected(selectedObject, intersectedObject);
                  if (selectedObject != null)
                  {
                     pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());
                  }
               }
            }
         }
      }
      else
      {
         mode = Mode.NONE;
         if (viewInput.isWindowHovered())
         {
            intersectedObject = calculatePickedObject(viewInput.getPickRayInWorld());

            if (intersectedObject != null && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               updateObjectSelected(selectedObject, intersectedObject);
               pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());
            }

//            if(viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
//            {
//               float value = 0.0f;
//               if (ImGui.beginChild("item context menu"))
//               {
//                  if(ImGui.beginPopupContextWindow())
//                  {
//                     if (ImGui.selectable("Copy")) value = 0.0f;
//                     if (ImGui.selectable("Paste")) value = 3.1415f;
//                     ImGui.endPopup();
//                  }
//                  ImGui.endChild();
//               }
//
//               LogTools.info("Click Registered: {}", value);
//            }
         }
      }
   }

   private GDXSimpleObject calculatePickedObject(Line3DReadOnly pickRay)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      GDXSimpleObject closestObject = null;

      if(building != null)
      {
         for (GDXSimpleObject object : building.getAllObjects())
         {
            boolean intersects = object.intersect(pickRay, tempIntersection);
            double distance = tempIntersection.distance(pickRay.getPoint());
            if (intersects && (closestObject == null || distance < closestDistance))
            {
               closestObject = object;
               closestDistance = distance;
            }
         }
      }
      return closestObject;
   }

   private void constructionUpdate(ImGui3DViewInput viewInput)
   {

      if (mode != Mode.NONE)
      {
         Line3DReadOnly pickRay = viewInput.getPickRayInWorld();
         lastPickPoint = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(EuclidCoreTools.origin3D,
                                                                                     Axis3D.Z,
                                                                                     pickRay.getPoint(),
                                                                                     pickRay.getDirection());
         switch (mode)
         {
            case CONSTRUCTING:
            {
               GDXSimpleObject objectToPlace = new GDXSimpleObject("Corner");
               Model objectModel = GDXModelBuilder.createCylinder(0.15f, 0.25f, Color.BROWN).model;
               Box3D collisionBox = new Box3D(0.25f, 0.25f, 0.15f);
               objectToPlace.setRealisticModel(objectModel);
//               objectToPlace.setCollisionModel(objectModel);
               objectToPlace.setCollisionGeometryObject(collisionBox);
               objectToPlace.setCollisionModelColor(highlightColor, 0.2f);
               virtualObjects.add(objectToPlace);
               updateObjectSelected(selectedObject, objectToPlace);

               mode = Mode.PLACING;
               break;
            }
            case PLACING:
            {
               cornerPoint = building.getClosestRectangularCorner(lastPickPoint);

               selectedObject.setPositionInWorld(cornerPoint);
               pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());

               break;
            }
            case DONE:
            {
               for (int i = 0; i < building.getCorners().size(); i++)
               {
                  Point3D corner = building.getCorners().get( (i + 1) % building.getCorners().size());
                  Point3D previousCorner = building.getCorners().get(i % building.getCorners().size());
                  double yaw = EuclidGeometryTools.angleFromFirstToSecondVector2D(corner.getX() - previousCorner.getX(),
                                                                                  corner.getY() - previousCorner.getY(),
                                                                                  1,
                                                                                  0);
                  float length = (float)EuclidGeometryTools.distanceBetweenPoint3Ds(corner.getX(),
                                                                                corner.getY(),
                                                                                corner.getZ(),
                                                                                previousCorner.getX(),
                                                                                previousCorner.getY(),
                                                                                previousCorner.getZ());
                  Point3D midPoint = new Point3D(0.0, 0.0, 0.0);
                  midPoint.add(corner);
                  midPoint.add(previousCorner);
                  midPoint.scale(0.5);


                  GDXSimpleObject objectToPlace = new GDXSimpleObject("BuildingWall_" + i);
                  Model objectModel = GDXModelBuilder.createBox(length, 0.1f, building.getHeight(), Color.LIGHT_GRAY).model;

                  Vector3DBasics translation = objectToPlace.getObjectTransform().getTranslation();
                  translationTransform.setTranslationAndIdentityRotation(translation);
                  objectToPlace.getObjectTransform().setRotationYawAndZeroTranslation(-yaw);
                  objectToPlace.getObjectTransform().multiply(translationTransform);
                  objectToPlace.setRealisticModel(objectModel);
//                  objectToPlace.setCollisionModel(objectModel);

                  Box3D collisionBox = new Box3D(length, 0.1f, building.getHeight());
                  objectToPlace.setCollisionGeometryObject(collisionBox);
                  objectToPlace.setCollisionModelColor(highlightColor, 0.2f);

                  objectToPlace.getCollisionShapeOffset().getTranslation().add(0.0f, 0.0f, building.getHeight() / 2.0f);
                  objectToPlace.getRealisticModelOffset().getTranslation().add(0.0f, 0.0f, building.getHeight() / 2.0f);
                  objectToPlace.setPositionInWorld(midPoint);
                  objectToPlace.getBoundingSphere().setRadius(building.getHeight() / 2.0f);

                  building.insertComponent(GDXBuildingObject.ComponentType.WALLS, objectToPlace);
               }

               selectedObject = null;
               virtualObjects.clear();
               mode = Mode.NONE;
               break;
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.separator();
      if (ImGui.sliderFloat("Ambient light", ambientLightAmount.getData(), 0.0f, 1.0f))
      {
         sceneManager.getSceneBasics().setAmbientLight(ambientLightAmount.get());
      }
      ImGui.separator();
      if (mode == Mode.NONE)
      {
         if (ImGui.button("Create Building"))
         {
            building = new GDXBuildingObject();
            mode = Mode.CONSTRUCTING;
         }

         if(ImGui.button("Create Stairs"))
         {
            constructStairwell(3, 4, 10, 3.0f, 3.0f, 12.0f);
         }

         ImGui.separator();
      }
      if (selectedObject != null && (ImGui.button("Delete selected") || ImGui.isKeyReleased(ImGuiTools.getDeleteKey())))
      {
         virtualObjects.remove(selectedObject);
         resetSelection();
      }

      ImGui.checkbox("Show 3D Widget Tuner", poseGizmoTunerPanel.getIsShowing());
   }

   public void updateObjectSelected(GDXSimpleObject from, GDXSimpleObject to)
   {
      if (from != to)
      {
         selectedObject = to;
      }
   }

   public void resetSelection()
   {
      updateObjectSelected(selectedObject, null);
      intersectedObject = null;
   }

   public void constructStairwell(int floors, int sides, int numberOfStepsPerSide, float length, float width, float height)
   {
      if(building == null) return;

      float[] xOffsets = {-width/2.0f, length/1.4f, width/2.0f, -length/1.4f};
      float[] yOffsets = {width/1.4f, length/2.0f, -width/1.4f, -length/2.0f};

      float[] xPlatformOffsets = {-width/2.0f, length/2.0f, width/2.0f, -length/2.0f};
      float[] yPlatformOffsets = {width/2.0f, length/2.0f, -width/2.0f, -length/2.0f};

      for(int i = 0; i<floors; i++)
      {
         for(int j = 0; j<sides; j++)
         {
            GDXSimpleObject stairsObject = new GDXSimpleObject("Stairs_" + ((sides * i) + j));
            Model objectModel = GDXModelBuilder.createStairs(1.5f, 0.3f, 0.3f, 10, Color.GRAY).model;
            stairsObject.setRealisticModel(objectModel);
            stairsObject.setCollisionModel(objectModel);

            Box3D collisionBox = new Box3D(1.0f, 0.1f, building.getHeight());
            stairsObject.setCollisionGeometryObject(collisionBox);
            stairsObject.getRealisticModelOffset().getRotation().appendYawRotation(-2.0f * Math.PI / (float)(sides) * j);

            stairsObject.getRealisticModelOffset().getTranslation().add(xOffsets[j], yOffsets[j], 3.0f * (float)(i*sides + j) - 0.3f);
            stairsObject.setPositionInWorld(new Point3D(0.0f, 0.0f, 0.0f));

            building.insertComponent(GDXBuildingObject.ComponentType.STAIRS, stairsObject);

            GDXSimpleObject platformObject = new GDXSimpleObject("Platform_" + ((sides * i) + j));
            Model platformModel = GDXModelBuilder.createBox(3.0f, 1.5f, 0.3f, Color.GRAY).model;
            platformObject.setRealisticModel(platformModel);
            platformObject.setCollisionModel(platformModel);

            Box3D platformCollisionBox = new Box3D(1.0f, 0.1f, building.getHeight());
            platformObject.setCollisionGeometryObject(platformCollisionBox);
            platformObject.getRealisticModelOffset().getRotation().appendYawRotation(-2.0f * Math.PI / (float)(sides) * j);

            platformObject.getRealisticModelOffset().getTranslation().add(xOffsets[j], yOffsets[j], 3.0f * (float)(i*sides + j) - 0.45f);
            platformObject.setPositionInWorld(new Point3D(0.0f, 0.0f, 0.0f));

            building.insertComponent(GDXBuildingObject.ComponentType.PLATFORMS, platformObject);
         }
      }
   }
}
