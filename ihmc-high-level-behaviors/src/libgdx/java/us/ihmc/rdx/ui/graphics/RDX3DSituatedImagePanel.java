package us.ihmc.rdx.ui.graphics;

@Deprecated
public class RDX3DSituatedImagePanel
{
//   private static final double FOLLOW_HEADSET_OFFSET_Y = 0.0;
//   private static final double FOLLOW_HEADSET_OFFSET_Z = 0.17;
//
//   private ModelInstance modelInstance;
//   private ModelInstance hoverBoxMesh;
//   private Texture texture;
//   private final FramePoint3D tempFramePoint = new FramePoint3D();
//   private final Vector3 topLeftPosition = new Vector3();
//   private final Vector3 bottomLeftPosition = new Vector3();
//   private final Vector3 bottomRightPosition = new Vector3();
//   private final Vector3 topRightPosition = new Vector3();
//   private final Vector3 topLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
//   private final Vector3 bottomLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
//   private final Vector3 bottomRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
//   private final Vector3 topRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
//   private final Vector2 topLeftUV = new Vector2();
//   private final Vector2 bottomLeftUV = new Vector2();
//   private final Vector2 bottomRightUV = new Vector2();
//   private final Vector2 topRightUV = new Vector2();
//
//   @Nullable
//   private final RDXVRModeManager vrModeManager;
//   private final MutableReferenceFrame floatingPanelFrame = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());
//   private final FramePose3D floatingPanelFramePose = new FramePose3D();
//   private double panelDistanceFromHeadset = 0.5;
//   private boolean isShowing = false;
//   private final FrameBox3D selectionCollisionBox = new FrameBox3D();
//   private final PointCollidable pointCollidable = new PointCollidable(selectionCollisionBox);
//   private final SideDependentList<RDXVRPickResult> vrPickResult = new SideDependentList<>(RDXVRPickResult::new);
//   /** If either VR controller is hovering the panel. */
//   private boolean isVRHovering = false;
//
//   /**
//    * Create for a programmatically placed panel.
//    */
//   public RDX3DSituatedImagePanel()
//   {
//      vrModeManager = null;
//   }
//
//   /**
//    * Create and enable VR interaction.
//    * @param vrModeManager TODO: Remove this and replace with context based manipulation
//    */
//   public RDX3DSituatedImagePanel(RDXVRContext context, RDXVRModeManager vrModeManager)
//   {
//      this.vrModeManager = vrModeManager;
//      context.addVRPickCalculator(this::calculateVRPick);
//      context.addVRInputProcessor(this::processVRInput);
//   }
//
//   public void create(Texture texture, Frustum frustum, ReferenceFrame centerOfPanelFrame, boolean flipY)
//   {
//      // Counter clockwise order
//      // Draw so thumb faces away and index right
//      Vector3[] planePoints = frustum.planePoints;
//      topLeftPosition.set(planePoints[7]);
//      bottomLeftPosition.set(planePoints[4]);
//      bottomRightPosition.set(planePoints[5]);
//      topRightPosition.set(planePoints[6]);
//      create(texture, centerOfPanelFrame, flipY);
//   }
//
//   public void create(Texture texture, Vector3[] points, ReferenceFrame centerOfPanelFrame, boolean flipY)
//   {
//      topLeftPosition.set(points[0]);
//      bottomLeftPosition.set(points[1]);
//      bottomRightPosition.set(points[2]);
//      topRightPosition.set(points[3]);
//      create(texture, centerOfPanelFrame, flipY);
//   }
//
//   public void create(Texture texture, double panelWidth, double panelHeight, ReferenceFrame centerOfPanelFrame, boolean flipY)
//   {
//      float halfPanelHeight = (float) panelHeight / 2.0f;
//      float halfPanelWidth = (float) panelWidth / 2.0f;
//      topLeftPosition.set(halfPanelHeight, halfPanelWidth, 0.0f);
//      bottomLeftPosition.set(-halfPanelHeight, halfPanelWidth, 0.0f);
//      bottomRightPosition.set(-halfPanelHeight, -halfPanelWidth, 0.0f);
//      topRightPosition.set(halfPanelHeight, -halfPanelWidth, 0.0f);
//      create(texture, centerOfPanelFrame, flipY);
//   }
//
//   private void create(Texture texture, ReferenceFrame centerOfPanelFrame, boolean flipY)
//   {
//      this.texture = texture;
//      ModelBuilder modelBuilder = new ModelBuilder();
//      modelBuilder.begin();
//
//      MeshBuilder meshBuilder = new MeshBuilder();
//      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);
//
//      // Counter clockwise order
//      // Draw so thumb faces away and index right
//      topLeftUV.set(0.0f, flipY ? 1.0f : 0.0f);
//      bottomLeftUV.set(0.0f, flipY ? 0.0f : 1.0f);
//      bottomRightUV.set(1.0f, flipY ? 0.0f : 1.0f);
//      topRightUV.set(1.0f, flipY ? 1.0f : 0.0f);
//
//      // Transform the corners into world frame
//      transformFromLocalToWorld(topLeftPosition, centerOfPanelFrame);
//      transformFromLocalToWorld(bottomLeftPosition, centerOfPanelFrame);
//      transformFromLocalToWorld(bottomRightPosition, centerOfPanelFrame);
//      transformFromLocalToWorld(topRightPosition, centerOfPanelFrame);
//
//      // Add vertices for the front of the panel
//      meshBuilder.vertex(topLeftPosition, topLeftNormal, Color.WHITE, topLeftUV);
//      meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, Color.WHITE, bottomLeftUV);
//      meshBuilder.vertex(bottomRightPosition, bottomRightNormal, Color.WHITE, bottomRightUV);
//      meshBuilder.vertex(topRightPosition, topRightNormal, Color.WHITE, topRightUV);
//
//      // Add a mirrored image on the back so it's always visible.
//      meshBuilder.vertex(topLeftPosition, topLeftNormal, Color.WHITE, topRightUV);
//      meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, Color.WHITE, bottomRightUV);
//      meshBuilder.vertex(bottomRightPosition, bottomRightNormal, Color.WHITE, bottomLeftUV);
//      meshBuilder.vertex(topRightPosition, topRightNormal, Color.WHITE, topLeftUV);
//
//      // Front
//      meshBuilder.triangle((short) 3, (short) 0, (short) 1);
//      meshBuilder.triangle((short) 1, (short) 2, (short) 3);
//      // Back
//      meshBuilder.triangle((short) 5, (short) 4, (short) 7);
//      meshBuilder.triangle((short) 7, (short) 6, (short) 5);
//
//      Mesh mesh = meshBuilder.end();
//      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
//      Material material = new Material();
//
//      material.set(TextureAttribute.createDiffuse(texture));
//      material.set(ColorAttribute.createDiffuse(new Color(0.68235f, 0.688235f, 0.688235f, 1.0f)));
//      modelBuilder.part(meshPart, material);
//
//      // TODO: Rebuild the model if the camera parameters change.
//      Model model = modelBuilder.end();
//      modelInstance = new ModelInstance(model);
//
//      selectionCollisionBox.getSize().set(0.05,
//                                          Math.abs(topRightPosition.y - topLeftPosition.y),
//                                          Math.abs(topRightPosition.y - bottomLeftPosition.y));
//
//      FramePoint3DBasics[] vertices = selectionCollisionBox.getVertices();
//      hoverBoxMesh = new ModelInstance(RDXModelBuilder.buildModel(boxMeshBuilder ->
//                                               boxMeshBuilder.addMultiLineBox(vertices, 0.0005, new Color(Color.WHITE))));
//
//      for (RobotSide side : RobotSide.values)
//         vrPickResult.get(side).setPickedObjectID(this, "3D Situated Image Panel");
//   }
//
//   private void transformFromLocalToWorld(Vector3 positionToTransform, ReferenceFrame localFrame)
//   {
//      tempFramePoint.setToZero(ReferenceFrame.getWorldFrame());
//      LibGDXTools.toEuclid(positionToTransform, tempFramePoint);
//      tempFramePoint.changeFrame(localFrame);
//      LibGDXTools.toLibGDX(tempFramePoint, positionToTransform);
//   }
//
//   public void update(Texture imageTexture)
//   {
//      if (vrModeManager != null)
//      {
//         // Prevent ever having an invisible panel out there, which is very confusing
//         // to the VR user when the controllers are colliding with and invisible box.
//         boolean somethingToShow = imageTexture != null || texture != null;
//         isShowing = somethingToShow && vrModeManager.getShowFloatingVideoPanel().get();
//      }
//
//      // Update the texture if necessary
//      if (isShowing && imageTexture != null && imageTexture != texture)
//      {
//         boolean flipY = false;
//         float multiplier = 2.0f;
//         float halfWidth = imageTexture.getWidth() / 10000.0f * multiplier;
//         float halfHeight = imageTexture.getHeight() / 10000.0f * multiplier;
//         create(imageTexture,
//                new Vector3[] {new Vector3(0.0f, halfWidth, halfHeight),
//                               new Vector3(0.0f, halfWidth, -halfHeight),
//                               new Vector3(0.0f, -halfWidth, -halfHeight),
//                               new Vector3(0.0f, -halfWidth, halfHeight)},
//                floatingPanelFrame.getReferenceFrame(),
//                flipY);
//      }
//
//      setPoseToReferenceFrame(floatingPanelFrame.getReferenceFrame());
//      selectionCollisionBox.getPose().set(floatingPanelFramePose);
//      floatingPanelFramePose.setFromReferenceFrame(floatingPanelFrame.getReferenceFrame());
//      isVRHovering = false;
//      updatePoses();
//   }
//
//   public void calculateVRPick(RDXVRContext vrContext)
//   {
//      if (isShowing)
//      {
//         for (RobotSide side : RobotSide.values)
//         {
//            vrContext.getController(side).runIfConnected(controller ->
//            {
//               boolean isInside = pointCollidable.collide(controller.getPickPointPose().getPosition());
//               if (isInside)
//               {
//                  vrPickResult.get(side).setHoveringCollsion(controller.getPickPointPose().getPosition(), pointCollidable.getClosestPointOnSurface());
//                  controller.addPickResult(vrPickResult.get(side));
//               }
//            });
//         }
//      }
//   }
//
//   public void processVRInput(RDXVRContext context)
//   {
//      if (isShowing)
//      {
//         RDXVRPanelPlacementMode placementMode = vrModeManager.getVideoPanelPlacementMode();
//
//         context.getHeadset().runIfConnected(headset ->
//         {
//            if (placementMode == FOLLOW_HEADSET
//            || (placementMode == MANUAL_PLACEMENT && vrModeManager.getShowFloatVideoPanelNotification().poll()))
//            {
//               floatingPanelFramePose.setToZero(headset.getXForwardZUpHeadsetFrame());
//               floatingPanelFramePose.getPosition().set(panelDistanceFromHeadset, FOLLOW_HEADSET_OFFSET_Y, FOLLOW_HEADSET_OFFSET_Z);
//               floatingPanelFramePose.changeFrame(ReferenceFrame.getWorldFrame());
//               floatingPanelFramePose.get(floatingPanelFrame.getTransformToParent());
//               floatingPanelFrame.getReferenceFrame().update();
//               updatePoses();
//            }
//         });
//
//         for (RobotSide side : RobotSide.values)
//         {
//            context.getController(side).runIfConnected(controller ->
//            {
//               boolean isHovering = controller.getSelectedPick() == vrPickResult.get(side);
//               isVRHovering |= isHovering;
//
//               if (placementMode == MANUAL_PLACEMENT)
//               {
//                  RDXVRDragData gripDragData = controller.getGripDragData();
//
//                  if (isHovering && gripDragData.getDragJustStarted())
//                  {
//                     gripDragData.setObjectBeingDragged(this);
//                     gripDragData.setInteractableFrameOnDragStart(floatingPanelFrame.getReferenceFrame());
//                  }
//
//                  if (gripDragData.isBeingDragged(this))
//                  {
//                     gripDragData.getDragFrame().getTransformToDesiredFrame(floatingPanelFrame.getTransformToParent(),
//                                                                            floatingPanelFrame.getReferenceFrame().getParent());
//                     floatingPanelFrame.getReferenceFrame().update();
//                     updatePoses();
//                  }
//               }
//               else if (controller.getGripActionData().x() > 0.9 && selectionCollisionBox.isPointInside(controller.getPickPointPose().getPosition()))
//               {
//                  vrModeManager.setVideoPanelPlacementMode(MANUAL_PLACEMENT);
//               }
//            });
//         }
//      }
//   }
//
//   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
//   {
//      if (isShowing && sceneLevels.contains(RDXSceneLevel.VIRTUAL))
//      {
//         if (modelInstance != null)
//            modelInstance.getRenderables(renderables, pool);
//         if (hoverBoxMesh != null && isVRHovering)
//            hoverBoxMesh.getRenderables(renderables, pool);
//      }
//   }
//
//   private void updatePoses()
//   {
//      setPoseToReferenceFrame(floatingPanelFrame.getReferenceFrame());
//   }
//
//   public void setPoseToReferenceFrame(ReferenceFrame referenceFrame)
//   {
//      if (modelInstance != null)
//         LibGDXTools.toLibGDX(referenceFrame.getTransformToRoot(), modelInstance.transform);
//      if (hoverBoxMesh != null)
//         LibGDXTools.toLibGDX(referenceFrame.getTransformToRoot(), hoverBoxMesh.transform);
//   }
//
//   public ModelInstance getModelInstance()
//   {
//      return modelInstance;
//   }
//
//   public Texture getTexture()
//   {
//      return texture;
//   }
}
