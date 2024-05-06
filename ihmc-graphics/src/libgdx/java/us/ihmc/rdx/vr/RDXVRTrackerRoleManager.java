package us.ihmc.rdx.vr;

import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.interaction.SixDoFSelection;
import us.ihmc.robotics.interaction.SphereRayIntersection;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.HashMap;
import java.util.Map;
import java.util.SortedSet;

public class RDXVRTrackerRoleManager
{
   private final RDXVRTracker tracker;
   private final RDXVRContext vrContext;
   private final Map<String, Boolean> roleActivationMap = new HashMap<>();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final Map<String, Boolean> changedColorRoleButton = new HashMap<>();
   private boolean queuePopupToOpen = false;

   private boolean isTrackerHovered = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final Point3D closestCollision = new Point3D();
   private SixDoFSelection closestCollisionSelection = null;
   private double closestCollisionDistance;
   private final SphereRayIntersection boundingSphereIntersection = new SphereRayIntersection();
   private final SphereRayIntersection modelSphereIntersection = new SphereRayIntersection();
   private RDXModelInstance uninitializedModelInstance;

   public RDXVRTrackerRoleManager(RDXVRContext vrContext, RDXVRTracker tracker)
   {
      this.tracker = tracker;
      this.vrContext = vrContext;

      RDX3DPanel panel3D = RDXBaseUI.getInstance().getPrimary3DPanel();
      panel3D.addImGui3DViewPickCalculator(this::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(this::process3DViewInput);
      panel3D.addImGuiOverlayAddition(this::renderTooltipAndContextMenu);

      SortedSet<String> roles = vrContext.getAvailableTrackerRoles();
      roles.forEach(role -> roleActivationMap.put(role, false));
      roles.forEach(role -> changedColorRoleButton.put(role, false));

      uninitializedModelInstance = new RDXModelInstance(tracker.getModelInstance().model);
      uninitializedModelInstance.setColor(ColorDefinitions.Red());
      uninitializedModelInstance.setOpacity(0.5f);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      boolean isWindowHovered = ImGui.isWindowHovered();
      ImGuiMouseDragData manipulationDragData = input.getMouseDragData(ImGuiMouseButton.Left);

      // Here we are trying to avoid unecessary computation in collision calculation by filtering out
      // some common scenarios where we don't need to calculate the pick, which can be expensive
      if (isWindowHovered && (!manipulationDragData.isDragging() || manipulationDragData.getDragJustStarted()))
      {
         // This part is happening when the user could presumably start a drag
         // on this gizmo at any time

         Line3DReadOnly pickRay = input.getPickRayInWorld();
         determineCurrentSelectionFromPickRay(pickRay);

         if (closestCollisionSelection != null)
         {
            pickResult.setDistanceToCamera(closestCollisionDistance);
            input.addPickResult(pickResult);
         }
      }
   }

   private void determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {
      closestCollisionSelection = null;
      closestCollisionDistance = Double.POSITIVE_INFINITY;

      // Optimization: Do one large sphere collision to avoid completely far off picks
      boundingSphereIntersection.update(0.2, tracker.getXForwardZUpTrackerFrame().getTransformToWorldFrame());
      if (boundingSphereIntersection.intersect(pickRay))
      {
         modelSphereIntersection.update(0.08, tracker.getXForwardZUpTrackerFrame().getTransformToWorldFrame());
         boolean hoveringCenterSphere = modelSphereIntersection.intersect(pickRay);
         double distance = modelSphereIntersection.getFirstIntersectionToPack().distance(pickRay.getPoint());
         if (hoveringCenterSphere && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            closestCollisionSelection = SixDoFSelection.CENTER;
            closestCollision.set(modelSphereIntersection.getFirstIntersectionToPack());
         }
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      isTrackerHovered = pickResult == input.getClosestPick();
      if (isTrackerHovered && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
      {
         queuePopupToOpen = true;
      }
   }

   private void renderTooltipAndContextMenu()
   {
      if (queuePopupToOpen)
      {
         queuePopupToOpen = false;
         ImGui.openPopup(labels.get("Popup"));
      }

      if (ImGui.beginPopup(labels.get("Popup")))
      {
         String formattedText = String.format("Tracker %d Role: ", tracker.getDeviceIndex() - 2); //remove the 2 controllers to get tracker index from SteamVR
         imgui.ImGui.text(formattedText);
         for (var role : roleActivationMap.entrySet())
         {
            if (vrContext.getAvailableTrackerRoles().contains(role.getKey()) || roleActivationMap.get(role.getKey()))
            {
               changedColorRoleButton.replace(role.getKey(), false);
               if (role.getValue())
               {
                  imgui.ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 0.0f, 1.0f, 0.5f);
                  changedColorRoleButton.replace(role.getKey(), true);
               }
               if (imgui.ImGui.button(labels.get(role.getKey()) + "##" + "Role"))
               {
                  role.setValue(!role.getValue());
                  if (role.getValue())
                  {
                     vrContext.setTrackerRole(role.getKey(), tracker.getDeviceIndex());
                     vrContext.setTrackerRoleAsUnavailable(role.getKey());
                     roleActivationMap.forEach((otherRoleKey, isActive) -> {
                        if (!otherRoleKey.equals(role.getKey()) && isActive)
                        {
                           roleActivationMap.put(otherRoleKey, false);
                           vrContext.setTrackerRoleAsAvailable(otherRoleKey);
                        }
                     });
                  }
                  else
                  {
                     vrContext.setTrackerRoleAsAvailable(role.getKey());
                  }
               }
               if (changedColorRoleButton.get(role.getKey()))
               {
                  imgui.ImGui.popStyleColor();
               }
            }
         }
         ImGui.separator();
         if(imgui.ImGui.button(labels.get("Reset All Roles")))
         {
            vrContext.resetTrackerRoles();
         }
         if (ImGui.menuItem("Close"))
            ImGui.closeCurrentPopup();
         ImGui.endPopup();
      }
   }

   public void reset()
   {
      SortedSet<String> roles = vrContext.getAvailableTrackerRoles();
      roles.forEach(role -> roleActivationMap.put(role, false));
      roles.forEach(role -> changedColorRoleButton.put(role, false));
   }

   public boolean isRoleAssigned()
   {
      for (var role : roleActivationMap.entrySet())
      {
         if (role.getValue())
            return true;
      }
      return false;
   }

   public RDXModelInstance getRedModelInstance()
   {
      uninitializedModelInstance.setPoseInWorldFrame(tracker.getDeviceYUpZBackFrame().getTransformToWorldFrame());
      return uninitializedModelInstance;
   }

   public int getTrackerIndex()
   {
      return tracker.getDeviceIndex();
   }
}
