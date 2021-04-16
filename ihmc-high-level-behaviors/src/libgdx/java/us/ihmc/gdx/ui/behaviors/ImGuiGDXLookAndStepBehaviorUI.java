package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.StoredPropertySetMessage;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.internal.flag.ImGuiItemFlags;
import com.badlogic.gdx.graphics.*;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiEnumPlot;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.editor.GDXUIActionMap;
import us.ihmc.gdx.input.editor.GDXUITrigger;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.graphics.GDXBodyPathPlanGraphic;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class ImGuiGDXLookAndStepBehaviorUI implements RenderableProvider
{
   private final BehaviorHelper behaviorHelper;

   private String currentState = "";
   private final ImBoolean operatorReview = new ImBoolean(true);
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 50);
   private long numberOfSteppingRegionsReceived = 0;
   private final ImGuiPlot steppingRegionsPlot = new ImGuiPlot("", 1000, 230, 30);

   private GDXImGuiBasedUI baseUI;
   private ModelInstance sphere;
   private ModelInstance arrow;
   private GDXUIActionMap placeGoalActionMap;
   private boolean placingGoal = false;
   private boolean placingPosition = true;
   private boolean reviewingBodyPath = true;
   private PlanarRegionsList latestRegions;
   private final Pose3D goalPose = new Pose3D();
   private final Point3D32 spherePosition = new Point3D32();
   private final Vector3D32 rotationVector = new Vector3D32();
   private final RotationMatrix arrowRotationMatrix = new RotationMatrix();

   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private final GDXBodyPathPlanGraphic bodyPathPlanGraphic = new GDXBodyPathPlanGraphic();
   private final GDXFootstepPlanGraphic footstepPlanGraphic = new GDXFootstepPlanGraphic();
   private final GDXFootstepPlanGraphic commandedFootstepsGraphic = new GDXFootstepPlanGraphic();
   private final GDXFootstepPlanGraphic startAndGoalFootstepsGraphic = new GDXFootstepPlanGraphic();

   private final ImGuiStoredPropertySetTuner lookAndStepParameterTuner
         = new ImGuiStoredPropertySetTuner(ImGuiTools.uniqueLabel(this, "Look and Step Parameters"));
   private final ImGuiStoredPropertySetTuner footstepPlannerParameterTuner
         = new ImGuiStoredPropertySetTuner(ImGuiTools.uniqueLabel(this, "Footstep Planner Parameters (for Look and Step)"));
   private final ImGuiStoredPropertySetTuner swingPlannerParameterTuner
         = new ImGuiStoredPropertySetTuner(ImGuiTools.uniqueLabel(this, "Swing Planner Parameters (for Look and Step)"));

   public ImGuiGDXLookAndStepBehaviorUI(BehaviorHelper behaviorHelper)
   {
      this.behaviorHelper = behaviorHelper;
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      this.baseUI = baseUI;

      if (behaviorHelper.getMessager() != null)
      {
         setupSubscribers();
      }

      LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
      lookAndStepParameterTuner.create(lookAndStepParameters,
                                       LookAndStepBehaviorParameters.keys,
                                       () ->
                                       {
                                          StoredPropertySetMessage storedPropertySetMessage = new StoredPropertySetMessage();
                                          lookAndStepParameters.getAllAsStrings().forEach(value -> storedPropertySetMessage.getStrings().add(value));
                                          behaviorHelper.publish(LOOK_AND_STEP_PARAMETERS, storedPropertySetMessage);
                                       });

      FootstepPlannerParametersBasics footstepPlannerParameters = behaviorHelper.getRobotModel().getFootstepPlannerParameters("ForLookAndStep");
      footstepPlannerParameterTuner.create(footstepPlannerParameters,
                                           FootstepPlannerParameterKeys.keys,
                                           () -> behaviorHelper.publish(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings()));

      SwingPlannerParametersBasics swingPlannerParameters = behaviorHelper.getRobotModel().getSwingPlannerParameters("ForLookAndStep");
      swingPlannerParameterTuner.create(swingPlannerParameters, SwingPlannerParameterKeys.keys,
                                        () -> behaviorHelper.publish(SwingPlannerParameters, swingPlannerParameters.getAllAsStrings()));

      float sphereRadius = 0.03f;
      sphere = GDXModelPrimitives.createSphere(sphereRadius, Color.CYAN);
      baseUI.getSceneManager().addRenderableProvider(sphere, GDXSceneLevel.VIRTUAL);

      arrow = GDXModelPrimitives.createArrow(sphereRadius * 6.0, Color.CYAN);
      baseUI.getSceneManager().addRenderableProvider(arrow, GDXSceneLevel.VIRTUAL);

      placeGoalActionMap = new GDXUIActionMap(startAction ->
      {
         placingGoal = true;
         placingPosition = true;
      });
      placeGoalActionMap.mapAction(GDXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         placingPosition = false;
      });
      placeGoalActionMap.mapAction(GDXUITrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         behaviorHelper.publish(GOAL_INPUT, goalPose);

         placingGoal = false;
      });
      placeGoalActionMap.mapAction(GDXUITrigger.RIGHT_CLICK, trigger ->
      {
         placingGoal = false;
      });

      baseUI.addImGui3DViewInputProcessor(this::processImGui3DViewInput);
   }

   public void setupSubscribers()
   {
      behaviorHelper.subscribeViaCallback(CurrentState, state -> currentState = state);
      behaviorHelper.subscribeViaCallback(OperatorReviewEnabledToUI, operatorReview::set);
      behaviorHelper.subscribeViaCallback(PlanarRegionsForUI, regions ->
      {
         this.latestRegions = regions;
         ++numberOfSteppingRegionsReceived;
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });
      behaviorHelper.subscribeViaCallback(BodyPathPlanForUI, bodyPath ->
      {
         if (bodyPath != null)
            bodyPathPlanGraphic.generateMeshesAsync(bodyPath);
      });
      footstepPlanGraphic.setTransparency(0.2);
      behaviorHelper.subscribeViaCallback(FootstepPlanForUI, footsteps ->
      {
         reviewingBodyPath = false;
         footstepPlanGraphic.generateMeshesAsync(footsteps);
      });
      behaviorHelper.subscribeViaCallback(LastCommandedFootsteps, commandedFootstepsGraphic::generateMeshesAsync);
      startAndGoalFootstepsGraphic.setColor(RobotSide.LEFT, Color.BLUE);
      startAndGoalFootstepsGraphic.setColor(RobotSide.RIGHT, Color.BLUE);
      startAndGoalFootstepsGraphic.setTransparency(0.4);
      behaviorHelper.subscribeViaCallback(StartAndGoalFootPosesForUI, startAndGoalFootstepsGraphic::generateMeshesAsync);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      if (placingGoal && input.isWindowHovered())
      {
         Line3DReadOnly pickRayInWorld = input.getPickRayInWorld(baseUI);
         PlanarRegionsList latestRegions = this.latestRegions;
         Point3D pickPoint = null;
         if (latestRegions != null)
         {
            for (PlanarRegion planarRegion : latestRegions.getPlanarRegionsAsList())
            {
               Point3D intersection = PlanarRegionTools.intersectRegionWithRay(planarRegion, pickRayInWorld.getPoint(), pickRayInWorld.getDirection());
               if (intersection != null)
               {
                  if (pickPoint == null)
                  {
                     pickPoint = intersection;
                  }
                  else
                  {
                     if (intersection.distance(pickRayInWorld.getPoint()) < pickPoint.distance(pickRayInWorld.getPoint()))
                     {
                        pickPoint = intersection;
                     }
                  }
               }
            }
         }

         if (pickPoint == null)
         {
            pickPoint = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(EuclidCoreTools.origin3D,
                                                                                              Axis3D.Z,
                                                                                              pickRayInWorld.getPoint(),
                                                                                              pickRayInWorld.getDirection());
         }

         if (placingPosition)
         {
            sphere.transform.setTranslation(pickPoint.getX32(), pickPoint.getY32(), pickPoint.getZ32());

            if (ImGui.isMouseReleased(ImGuiMouseButton.Left))
            {
               placeGoalActionMap.triggerAction(GDXUITrigger.POSITION_LEFT_CLICK);
            }
         }
         else // placing orientation
         {
            GDXTools.toEuclid(sphere.transform, spherePosition);
            GDXTools.toGDX(spherePosition, arrow.transform);

            rotationVector.set(pickPoint);
            rotationVector.sub(spherePosition);

            double yaw = Math.atan2(rotationVector.getY(), rotationVector.getX());
            arrowRotationMatrix.setToYawOrientation(yaw);
            GDXTools.toGDX(arrowRotationMatrix, arrow.transform);

            goalPose.set(spherePosition, arrowRotationMatrix);

            if (ImGui.isMouseReleased(ImGuiMouseButton.Left))
            {
               placeGoalActionMap.triggerAction(GDXUITrigger.ORIENTATION_LEFT_CLICK);
            }
         }

         if (ImGui.isMouseReleased(ImGuiMouseButton.Right))
         {
            placeGoalActionMap.triggerAction(GDXUITrigger.RIGHT_CLICK);
         }
      }
   }

   public void render()
   {
      ImGui.begin(getWindowName());

      ImGui.text("Current state:");
      if (!currentState.isEmpty())
      {
         LookAndStepBehavior.State state = LookAndStepBehavior.State.valueOf(currentState);
         currentStatePlot.render(state.ordinal(), state.name());
      }
      else
      {
         currentStatePlot.render(-1, "");
      }

      if (ImGui.button("Select behavior"))
      {
         behaviorHelper.publish(BehaviorModule.API.BehaviorSelection, LookAndStepBehavior.DEFINITION.getName());
      }
      ImGui.sameLine();
      if (ImGui.button("Reset"))
      {
         behaviorHelper.publish(RESET);
      }
      ImGui.sameLine();

      boolean pushed = false;
      if (placingGoal)
      {
         pushed = true;
         ImGui.pushItemFlag(ImGuiItemFlags.Disabled, true);
      }
      if (ImGui.button("Place goal"))
      {
         placeGoalActionMap.start();
      }
      if (pushed)
      {
         ImGui.popItemFlag();
      }

      if (ImGui.checkbox("Operator review", operatorReview))
      {
         behaviorHelper.publish(OperatorReviewEnabled, operatorReview.get());
      }
      if (ImGui.button("Reject"))
      {
         behaviorHelper.publish(ReviewApproval, false);
      }
      ImGui.sameLine();
      if (ImGui.button("Approve"))
      {
         behaviorHelper.publish(ReviewApproval, true);
      }
      ImGui.text("Footstep planning regions recieved:");
      steppingRegionsPlot.render(numberOfSteppingRegionsReceived);

      ImGui.end();

      lookAndStepParameterTuner.render();
      footstepPlannerParameterTuner.render();
      swingPlannerParameterTuner.render();

      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.render();
         commandedFootstepsGraphic.render();
         startAndGoalFootstepsGraphic.render();
         planarRegionsGraphic.render();
         bodyPathPlanGraphic.render();
      }
   }

   private boolean areGraphicsEnabled()
   {
      return !currentState.isEmpty() && !currentState.equals(LookAndStepBehavior.State.RESET.name());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.getRenderables(renderables, pool);
         commandedFootstepsGraphic.getRenderables(renderables, pool);
         startAndGoalFootstepsGraphic.getRenderables(renderables, pool);
         planarRegionsGraphic.getRenderables(renderables, pool);
         bodyPathPlanGraphic.getRenderables(renderables, pool);
      }
   }

   public void setMessager(Messager messager)
   {
      behaviorHelper.setNewMessager(messager);
      setupSubscribers();
   }

   public void destroy()
   {
      footstepPlanGraphic.destroy();
      commandedFootstepsGraphic.destroy();
      startAndGoalFootstepsGraphic.destroy();
      planarRegionsGraphic.destroy();
      bodyPathPlanGraphic.destroy();
   }

   public String getWindowName()
   {
      return LookAndStepBehavior.DEFINITION.getName();
   }
}
