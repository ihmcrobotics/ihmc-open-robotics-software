package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Vector3;
import imgui.flag.ImGuiKey;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXLargeCinderBlockRoughed;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

import static us.ihmc.gdx.simulation.environment.GDXModelInput.State.*;

public class GDXModelInput
{

   public static enum State
   {
      PLACING_XY, PLACING_X, PLACING_Y, PLACING_Z, ROTATING_YAW, ROTATING_PITCH, ROTATING_ROLL, NONE
   }

   private float previousStateMouseX = 0;
   private float previousStateMouseY = 0;
   private float mouseVelX = 0;
   private float mouseVelY = 0;

   private ArrayList<GDXEnvironmentObject> environmentObjects = new ArrayList<>();
   private HashSet<Integer> selectedObjectIndexes = new HashSet<>();
   private HashMap<Integer, Material> originalMaterials = new HashMap<>();

   public State state = NONE;

   private RigidBodyTransform tempRigidBodyTransform = new RigidBodyTransform();
   private Vector3 tempModelTranslation = new Vector3();
   private Point3D translation = new Point3D();

   private Point3D modelPosition = new Point3D();
   private Vector3D cameraOrigin = new Vector3D();
   private Vector3D originToPosition = new Vector3D();

   private HashMap<Character, ModelInstance> controlMap = new HashMap<>();
   private HashSet<ModelInstance> controlAxes = new HashSet<>();
   private final Material selectionMaterial;
   private float modelYaw, modelPitch, modelRoll = 0;
   private boolean editMode = false;
   private GDXImGuiBasedUI baseUI;
   private FramePose3D tempFramePose = new FramePose3D();

   public GDXModelInput()
   {
      selectionMaterial = new Material();
      selectionMaterial.set(ColorAttribute.createDiffuse(Color.ORANGE));
   }

   public void create()
   {
      Gdx.gl.glLineWidth(8);
      ModelBuilder modelBuilder = new ModelBuilder();

      modelBuilder.begin();
      MeshPartBuilder builder = modelBuilder.part("z_axis",
                                                  GL20.GL_LINES,
                                                  VertexAttributes.Usage.Position | VertexAttributes.Usage.ColorUnpacked,
                                                  new Material());
      builder.setColor(Color.BLUE);
      builder.line(0.0f, 0.0f, -20.0f, 0.0f, 0.0f, 20.0f);
      Model zLineModel = modelBuilder.end();
      ModelInstance zLineInstance = new ModelInstance(zLineModel);

      modelBuilder.begin();
      builder = modelBuilder.part("y_axis", GL20.GL_LINES, VertexAttributes.Usage.Position | VertexAttributes.Usage.ColorUnpacked, new Material());
      builder.setColor(Color.YELLOW);
      builder.line(0.0f, -20.0f, 0.0f, 0.0f, 20.0f, 0.0f);
      Model yLineModel = modelBuilder.end();
      ModelInstance yLineInstance = new ModelInstance(yLineModel);

      modelBuilder.begin();
      builder = modelBuilder.part("x_axis", GL20.GL_LINES, VertexAttributes.Usage.Position | VertexAttributes.Usage.ColorUnpacked, new Material());
      builder.setColor(Color.RED);
      builder.line(-20.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f);
      Model xLineModel = modelBuilder.end();
      ModelInstance xLineInstance = new ModelInstance(xLineModel);

      controlMap.put('Z', zLineInstance);
      controlMap.put('Y', yLineInstance);
      controlMap.put('X', xLineInstance);
   }

   public void updateSelections(ImGui3DViewInput input)
   {
      Line3DReadOnly pickRayInWorld = input.getPickRayInWorld(baseUI);
      int result = -1;
      double finalPerpDist = Double.MAX_VALUE;
      double finalDistFromOrigin = Double.MAX_VALUE;

      for (int i = 0; i < environmentObjects.size(); i++)
      {
         final GDXEnvironmentObject environmentObject = environmentObjects.get(i);
         environmentObject.getModelInstance().transform.getTranslation(tempModelTranslation);

         GDXTools.toEuclid(tempModelTranslation, modelPosition);
         cameraOrigin.set(pickRayInWorld.getPointX(), pickRayInWorld.getPointY(), pickRayInWorld.getPointZ());
         originToPosition.sub(modelPosition, cameraOrigin);

         double distFromOrigin = pickRayInWorld.getDirection().dot(originToPosition);
         double perpDist = pickRayInWorld.distance(modelPosition);


         if (perpDist <= 0.8 * environmentObject.getModelInstance().radius)
         {
            result = i;
            finalPerpDist = perpDist;
            finalDistFromOrigin = distFromOrigin;
         }
      }

      if (ImGui.isMouseClicked(ImGuiMouseButton.Left) && editMode)
      {
         if (result != -1 && !selectedObjectIndexes.contains(result))
         {
            selectedObjectIndexes.add(result);
            Material origMat = new Material(environmentObjects.get(result).getModelInstance().materials.get(0));
            originalMaterials.put(result, origMat);
            environmentObjects.get(result).getModelInstance().materials.get(0).set(selectionMaterial);
         }
         else if (result == -1)
         {
            if (ImGui.isMouseClicked(ImGuiMouseButton.Left))
            {
               clearSelections();
            }
         }
      }
      if (ImGui.isMouseClicked(ImGuiMouseButton.Middle))
      {
         duplicateSelections();
      }
   }

   public void duplicateSelections()
   {
      ArrayList<Integer> duplicates = new ArrayList<>(selectedObjectIndexes);

      clearSelections();
      for (int index : duplicates)
      {
         GDXEnvironmentObject duplicate = environmentObjects.get(index).duplicate();
         environmentObjects.add(duplicate);
         selectedObjectIndexes.add(environmentObjects.size() - 1);
         Material origMat = new Material(duplicate.getModelInstance().materials.get(0));
         originalMaterials.put(environmentObjects.size() - 1, origMat);
         duplicate.getModelInstance().materials.get(0).set(selectionMaterial);
      }
   }

   public void clearSelections()
   {
      for (int i : selectedObjectIndexes)
      {
         System.out.println("Clearing:" + i);
         environmentObjects.get(i).getModelInstance().materials.get(0).set(originalMaterials.get(i));
      }
      selectedObjectIndexes.clear();
      originalMaterials.clear();
   }

   public void updateState(ImGui3DViewInput input)
   {
      if (editMode)
      {
         if (ImGui.isKeyDown('B'))
         {
//            controlAxes.add(controlMap.get('Z'));
            state = PLACING_Z;
            if (ImGui.isKeyDown(' '))
               state = ROTATING_YAW;
         }
         else if (ImGui.isKeyDown('C'))
         {
//            controlAxes.add(controlMap.get('X'));
            state = PLACING_X;
            if (ImGui.isKeyDown(' '))
               state = ROTATING_ROLL;
         }
         else if (ImGui.isKeyDown('V'))
         {
//            controlAxes.add(controlMap.get('Y'));
            state = PLACING_Y;
            if (ImGui.isKeyDown(' '))
               state = ROTATING_PITCH;
         }
         else if (ImGui.isKeyPressed(ImGuiKey.Insert))
         {

         }
         else
         {
            state = NONE;
            controlAxes.clear();
         }
      }
   }

   public void updatePoseForSelections(ImGui3DViewInput input)
   {
      if (input.isWindowHovered())
      {
         LogTools.debug(state + "\t" + selectedObjectIndexes + "\tTotal Materials: " + originalMaterials.size() + "\tControls: " + controlAxes.size());

         translation.setToZero();
         modelRoll = modelYaw = modelPitch = 0;

         if (ImGui.isMouseDown(ImGuiMouseButton.Right) || state == PLACING_XY)
         {
            mouseVelX = previousStateMouseX - input.getMousePosX();
            mouseVelY = previousStateMouseY - input.getMousePosY();

            previousStateMouseX = input.getMousePosX();
            previousStateMouseY = input.getMousePosY();

            Line3DReadOnly pickRayInWorld = input.getPickRayInWorld(baseUI);

            switch (state)
            {
               case PLACING_XY:
                  Point3D pickPointGround = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(EuclidCoreTools.origin3D,
                                                                                                    Axis3D.Z,
                                                                                                    pickRayInWorld.getPoint(),
                                                                                                    pickRayInWorld.getDirection());

                  pickPointGround.addZ(0.08f);
                  tempRigidBodyTransform.setTranslationAndIdentityRotation(pickPointGround);
                  tempRigidBodyTransform.appendPitchRotation((float) Math.toRadians(90.0));
                  if (ImGui.isMouseClicked(ImGuiMouseButton.Left))
                  {
                     state = NONE;
                     selectedObjectIndexes.clear();
                     originalMaterials.clear();
                  }
                  break;

               case PLACING_X:
                  translation.setX(0.01 * mouseVelY);
                  break;
               case PLACING_Y:
                  translation.setY(0.01 * mouseVelY);
                  break;
               case PLACING_Z:
                  translation.setZ(0.01 * mouseVelY);
                  break;
               case ROTATING_YAW:
                  modelYaw = (float) Math.toRadians(mouseVelY) * 4;
                  break;
               case ROTATING_PITCH:
                  modelPitch = (float) Math.toRadians(mouseVelY) * 4;
                  break;
               case ROTATING_ROLL:
                  modelRoll = (float) Math.toRadians(mouseVelY) * 4;
                  break;
            }
         }
         else if (ImGui.isMouseReleased(ImGuiMouseButton.Right))
         {
            translation.setToZero();
            mouseVelX = mouseVelY = 0;
         }
         transformSelections();
      }
   }

   public void transformSelections(){
      for (int selection : selectedObjectIndexes)
      {
         GDXEnvironmentObject selectedModel = environmentObjects.get(selection);

         selectedModel.getModelInstance().transform.getTranslation(tempModelTranslation);


         if (state == PLACING_XY)
         {
            GDXTools.toGDX(tempRigidBodyTransform, selectedModel.getModelInstance().transform);
         }
         else
         {
            GDXTools.toEuclid(selectedModel.getModelInstance().transform, tempRigidBodyTransform);
            tempRigidBodyTransform.prependTranslation(translation);
            tempRigidBodyTransform.prependPitchRotation(modelPitch);
            tempRigidBodyTransform.prependYawRotation(modelYaw);
            tempRigidBodyTransform.prependRollRotation(modelRoll);
            tempRigidBodyTransform.normalizeRotationPart();
            GDXTools.toGDX(tempRigidBodyTransform, selectedModel.getModelInstance().transform);
         }
      }
   }

   public void handleVREvents(GDXVRManager vrManager)
   {
      if (vrManager.isVREnabled() && !selectedObjectIndexes.isEmpty())
      {
         PoseReferenceFrame controllerFrame = vrManager.getControllers().get(RobotSide.RIGHT).getReferenceFrame();
         tempFramePose.setToZero(controllerFrame);
         tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         tempFramePose.get(tempRigidBodyTransform);
         GDXTools.toGDX(tempRigidBodyTransform, environmentObjects.get(selectedObjectIndexes.stream().findFirst().get()).getModelInstance().transform);
      }
   }

   public void addLargeCinderBlockRoughed()
   {
      addAndSelectInstance(new GDXLargeCinderBlockRoughed());
   }

   public void addAndSelectInstance(GDXEnvironmentObject environmentObject)
   {
      clearSelections();
      environmentObjects.add(environmentObject);
      selectedObjectIndexes.add(environmentObjects.size() - 1);
   }

   public void addInstance(GDXEnvironmentObject instance)
   {
      environmentObjects.add(instance);
   }

   public ArrayList<GDXEnvironmentObject> getEnvironmentObjects()
   {
      return environmentObjects;
   }

   public void clear()
   {
      environmentObjects.clear();
      selectedObjectIndexes.clear();
      originalMaterials.clear();
   }

   public boolean isDone()
   {
      return state == NONE && !editMode;
   }

   public void setState(State state)
   {
      this.state = state;
   }

   public void setBaseUI(GDXImGuiBasedUI baseUI)
   {
      this.baseUI = baseUI;
   }

   public void setEditMode(boolean editMode)
   {
      this.editMode = editMode;
   }

   public HashSet<ModelInstance> getControlAxes()
   {
      return controlAxes;
   }
}
