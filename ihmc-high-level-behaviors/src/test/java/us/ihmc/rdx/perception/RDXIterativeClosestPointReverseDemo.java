package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.opencv.global.opencv_core;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import org.ejml.simple.SimpleSVD;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXArUcoBoxObject;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class RDXIterativeClosestPointReverseDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   private RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private ModelInstance mousePickSphere;
   private ModelInstance box;
   private Point3DReadOnly pickPointInWorld = new Point3D();
   private RDXBytedecoImagePanel mainViewDepthPanel;
   private BytedecoImage image;
   private final ImBoolean ICPToggle = new ImBoolean(true);
   private final RecyclingArrayList<Point3D32> envPointCloud = new RecyclingArrayList<>(Point3D32::new);
   private final RecyclingArrayList<FramePoint3D> boxModelPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<Point3D32> boxInWorldPoints = new RecyclingArrayList<>(Point3D32::new);
   private List<Float> ICPDistances = new ArrayList<Float>();
   private List<Integer> ICPIndices = new ArrayList<Integer>();


   private final RDXPointCloudRenderer envPointCloudRenderer = new RDXPointCloudRenderer();
   private final RDXPointCloudRenderer boxPointCloudRenderer = new RDXPointCloudRenderer();

   private RigidBodyTransform boxTransform = new RigidBodyTransform();
   private ReferenceFrame boxReferenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), boxTransform);
   private FramePoint3D pointA = new FramePoint3D(boxReferenceFrame);

   float boxSize = 0.35f;
   float boxHalfSize = boxSize/2;

   private final int numDimensions = 3;
   private final int maxIterations = 5;
   private final float tolerance = 0.001f;
   float distance = 0.0f;
   private DMatrixRMaj boxCentroid;//  = new DMatrixRMaj(1, 3);
   private DMatrixRMaj envCentroid;//  = new DMatrixRMaj(1, 3);
   private DMatrixRMaj boxCentroidSubtractedPoints;// = new DMatrixRMaj(1, 3);
   private DMatrixRMaj envCentroidSubtractedPoints;// = new DMatrixRMaj(1, 3);
   private DMatrixRMaj boxCorrespondencePoints;
   private DMatrixRMaj zeroMatrixPoint = new DMatrixRMaj(1,3);

   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);
   private int envSize = 6;
   private Point3D testPointInWorld = new Point3D(2.5, 0.0, 1.0);
   private RDXEnvironmentObject realBox;
   private Pose3D realBoxPose = new Pose3D();;
   private long counter = 0;



   public RDXIterativeClosestPointReverseDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            // Create base GUI
            baseUI.create();
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-3.0, 0.0, 2.5);

            // Create Environment and render panels
            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("HarderTerrain.json");
            realBox = environmentBuilder.getAllObjects().get(3);
            // Place sensor in environment
            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            // Adjust sensor position to better see the box
            sensorPoseGizmo.getTransformToParent().appendTranslation(0., 0., 5.);
            sensorPoseGizmo.getTransformToParent().appendYawRotation(0.0 * Math.PI / 180.0);
            sensorPoseGizmo.getTransformToParent().appendPitchRotation(60. * Math.PI / 180.0);

            // Set depth camera meta data
            // https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix
            double publishRateHz = 5.0;
            double verticalFOV = 55.0;
            int imageWidth = 640;
            int imageHeight = 480;
            double minRange = 0.105;
            double maxRange = 50.0;
            highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("Stepping L515",
                                                                                 sensorPoseGizmo.getGizmoFrame(),
                                                                                 () -> 0L,
                                                                                 verticalFOV,
                                                                                 imageWidth,
                                                                                 imageHeight,
                                                                                 minRange,
                                                                                 maxRange,
                                                                                 0.00,//0.03
                                                                                 0.00,//0.05
                                                                                 true,
                                                                                 publishRateHz);

            // Some BS that Duncan had and, I dont want to fuck around with, because I trust him.
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.setPublishPointCloudROS2(false);
            highLevelDepthSensorSimulator.setRenderPointCloudDirectly(true);
            highLevelDepthSensorSimulator.setPublishDepthImageROS1(false);
            highLevelDepthSensorSimulator.setDebugCoordinateFrame(true);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            highLevelDepthSensorSimulator.setRenderDepthVideoDirectly(true);
            highLevelDepthSensorSimulator.setPublishColorImageROS1(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS2(false);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator::getRenderables);

            // create red sphere for mouse localizer
            mousePickSphere = RDXModelBuilder.createSphere(0.03f, Color.RED);
            baseUI.getPrimaryScene().addRenderableProvider(mousePickSphere, RDXSceneLevel.VIRTUAL);

            // create box for icp
            box = RDXModelBuilder.createBox(boxSize, boxSize, boxSize, new Color(0x4169e100));
            LibGDXTools.setOpacity(box, 0.1f);
            baseUI.getPrimaryScene().addRenderableProvider(box, RDXSceneLevel.VIRTUAL);



            // Create point cloud renderer for icp points
            baseUI.getPrimaryScene().addRenderableProvider(envPointCloudRenderer::getRenderables, RDXSceneLevel.VIRTUAL);
            envPointCloudRenderer.create(300000);

            // Create point cloud and renderer for box points
            Random random = new Random(0);
//            random.nextDouble(-boxHalfSize, boxHalfSize);
//            float halfBoxWidth = (float)RigidBodySceneObjectDefinitions.BOX_WIDTH/2.0f;
//            float halfBoxDepth = (float)RigidBodySceneObjectDefinitions.BOX_DEPTH/2.0f;
//            float halfBoxHeight = (float)RigidBodySceneObjectDefinitions.BOX_HEIGHT/2.0f;
//            for (int i = 0; i < 6; i++) {
//               for (int j = 0; j < 600; j++) {
//                  float x =(float)random.nextDouble(-halfBoxDepth, halfBoxDepth);
//                  float y =(float)random.nextDouble(-halfBoxWidth, halfBoxWidth);
//                  float z =(float)random.nextDouble(-halfBoxHeight, halfBoxHeight);
//                  if (i==0 | i==1) {x = (-((i >> 0)&1)*halfBoxDepth*2.0f)+halfBoxDepth;}
//                  if (i==2 | i==3) {y = (-((i >> 0)&1)*halfBoxWidth*2.0f)+halfBoxWidth;}
//                  if (i==4 | i==5) {z = (-((i >> 0)&1)*halfBoxHeight*2.0f)+halfBoxHeight;}
//                  pointA.setToZero(boxReferenceFrame);
//                  pointA.set(x,y,z);
//                  FramePoint3D worldFramePoint = boxModelPoints.add();
//                  worldFramePoint.setIncludingFrame(pointA);
//               }
//            }
            for (int i = 0; i < 6; i++) {
               float x = -0.5f+(0.2f*i);
               float y = 0.0f;
               float z = 0.0f;
               pointA.setToZero(boxReferenceFrame);
               pointA.set(x,y,z);
               FramePoint3D worldFramePoint = boxModelPoints.add();
               worldFramePoint.setIncludingFrame(pointA);
            }

            RDXInteractableReferenceFrame interactableReferenceFrame = new RDXInteractableReferenceFrame();
            interactableReferenceFrame.create(boxReferenceFrame, boxTransform, 1.0, baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableReferenceFrame::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableReferenceFrame::process3DViewInput);
//            baseUI.getPrimaryScene().addRenderableProvider(interactableReferenceFrame::getVirtualRenderables);
            baseUI.getPrimaryScene().addRenderableProvider(boxPointCloudRenderer::getRenderables, RDXSceneLevel.VIRTUAL);

            // Pick 3D point when collision is selected
//            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::ICPMainFunction);

            // Create ICP panel
            baseUI.getImGuiPanelManager().addPanel("Iterative Closest Point", this::renderImGuiICPWidgets);

            // create extra 3d panel
            RDX3DPanel panel3D = new RDX3DPanel("3D View 2", true);
            baseUI.add3DPanel(panel3D);

            // Initialize ICP distances and indices
            for (int i = 0; i < envSize; i++) {
               ICPDistances.add(Float.MAX_VALUE);
               ICPIndices.add(0);
            }

            boxCentroid  = new DMatrixRMaj(1, 3);
            envCentroid  = new DMatrixRMaj(1, 3);
            boxCentroidSubtractedPoints = new DMatrixRMaj(envSize, 3);
            boxCorrespondencePoints = new DMatrixRMaj(envSize, 3);
            envCentroidSubtractedPoints = new DMatrixRMaj(envSize, 3);

            getBoxPoints();
          }



         private void ICPMainFunction() {
            float cuttoffRange = .5f;
//            pickPointInWorld = input.getPickPointInWorld();
            LibGDXTools.toLibGDX(testPointInWorld, mousePickSphere.transform);

            envPointCloud.clear();
            if (ICPToggle.get()) {
               List<Point3D> depthData = highLevelDepthSensorSimulator.getPointCloud();

               // Seclect env point cloud and filter based on distance
//               for (int i = 0; i < depthData.size(); i++) {
//                  Point3D envPoint = depthData.get(i);
//                  double pointDifferenceFromSelected = testPointInWorld.distance(envPoint);
//                  if (pointDifferenceFromSelected <= cuttoffRange) {
//                     envPointCloud.add().set(envPoint);
//                  }
//               }

               for (int i = 0; i < 6; i++) {
                  float x = 1.0f;
                  float y = 0.5f+(0.2f*i);
                  float z = 1.0f;
                  envPointCloud.add().set(x,y,z);
               }





               // START ICP CALCS
               // Calc nearest neighbor env to object
               if (envPointCloud.size() >= envSize) {
                  envPointCloud.shuffle(new Random(0));
                  for (int i = 0; i < envSize; i++) {
                     ICPDistances.set(i, Float.MAX_VALUE);
                     ICPIndices.set(i, 0);
                  }
                  for (int i = 0; i < envSize; i++) {
                     Point3D32 envPoint = envPointCloud.get(i);
                     float minDistance = Float.MAX_VALUE;
                     int minIndice = Integer.MAX_VALUE;

                     for (int j = 0; j < boxModelPoints.size(); j++) {
                        distance = (float) envPoint.distance(boxInWorldPoints.get(j));
                        if (distance <= minDistance) {
                           minDistance = distance;
                           minIndice = j;
                        }
                     }
                     ICPDistances.set(i, minDistance);
                     ICPIndices.set(i, minIndice);
                     boxCorrespondencePoints.set(i, 0, boxInWorldPoints.get(minIndice).getX());
                     boxCorrespondencePoints.set(i, 1, boxInWorldPoints.get(minIndice).getY());
                     boxCorrespondencePoints.set(i, 2, boxInWorldPoints.get(minIndice).getZ());
                  }


                  // Calculate data centroid and arrange in matrices
                  boxCentroid.set(zeroMatrixPoint);
                  for (int i = 0; i < envSize; i++) {
                     boxCentroid.add(0, 0, boxCorrespondencePoints.get(i, 0));
                     boxCentroid.add(0, 1, boxCorrespondencePoints.get(i, 1));
                     boxCentroid.add(0, 2, boxCorrespondencePoints.get(i, 2));
                  }
                  boxCentroid.set(0, 0, boxCentroid.get(0, 0) / boxCorrespondencePoints.numRows);
                  boxCentroid.set(0, 1, boxCentroid.get(0, 1) / boxCorrespondencePoints.numRows);
                  boxCentroid.set(0, 2, boxCentroid.get(0, 2) / boxCorrespondencePoints.numRows);

                  for (int i = 0; i < envSize; i++) {
                     boxCentroidSubtractedPoints.set(i, 0, boxCorrespondencePoints.get(i, 0) - boxCentroid.get(0, 0));
                     boxCentroidSubtractedPoints.set(i, 1, boxCorrespondencePoints.get(i, 1) - boxCentroid.get(0, 1));
                     boxCentroidSubtractedPoints.set(i, 2, boxCorrespondencePoints.get(i, 2) - boxCentroid.get(0, 2));
                  }

                  // Calculate env centroid
                  envCentroid.set(zeroMatrixPoint);
                  for (int i = 0; i < envSize; i++) {
                     envCentroid.add(0, 0, envPointCloud.get(i).getX());
                     envCentroid.add(0, 1, envPointCloud.get(i).getY());
                     envCentroid.add(0, 2, envPointCloud.get(i).getZ());
                  }
                  envCentroid.set(0, 0, envCentroid.get(0, 0) / envSize);
                  envCentroid.set(0, 1, envCentroid.get(0, 1) / envSize);
                  envCentroid.set(0, 2, envCentroid.get(0, 2) / envSize);

                  for (int i = 0; i < envSize; i++) {
                     envCentroidSubtractedPoints.set(i, 0, envPointCloud.get(i).getX() - envCentroid.get(0, 0));
                     envCentroidSubtractedPoints.set(i, 1, envPointCloud.get(i).getY() - envCentroid.get(0, 1));
                     envCentroidSubtractedPoints.set(i, 2, envPointCloud.get(i).getZ() - envCentroid.get(0, 2));
                  }

                  // Initialize matrix variables
                  DMatrixRMaj H = new DMatrixRMaj(3, 3);
                  DMatrixRMaj U = new DMatrixRMaj(3, 3);
                  DMatrixRMaj V = new DMatrixRMaj(3, 3);
                  DMatrixRMaj R = new DMatrixRMaj(3, 3);
                  DMatrixRMaj newBoxLocation = new DMatrixRMaj(1, 3);
                  DMatrixRMaj boxTranslation = new DMatrixRMaj(1, 3);
                  DMatrixRMaj T = new DMatrixRMaj(4, 4);
                  CommonOps_DDRM.setIdentity(T);












                  // Set up test points
//                  DMatrixRMaj testObjPoints = new DMatrixRMaj(6, 3);
//                  DMatrixRMaj testEnvPoints = new DMatrixRMaj(6, 3);
//
//                  for (int i = 0; i < 6; i++) {
//                     testObjPoints.set(i, 0, -0.5+(0.2*i));
//                     testObjPoints.set(i, 1, 0.0);
//                     testObjPoints.set(i, 2, 0.0);
//
//                     testEnvPoints.set(i, 0, 0.0);
//                     testEnvPoints.set(i, 1, -0.5+(0.2*i));
//                     testEnvPoints.set(i, 2, 0.0);
//                  }
//
//                  DMatrixRMaj testboxCentroid = new DMatrixRMaj(1, 3);
//                  DMatrixRMaj testenvCentroid = new DMatrixRMaj(1, 3);
//
//                  testboxCentroid.set(0,0,0.5);
//                  testboxCentroid.set(0,1,0.0);
//                  testboxCentroid.set(0,2,0.0);
//
//                  testenvCentroid.set(0,0,1.0);
//                  testenvCentroid.set(0,1,1.5);
//                  testenvCentroid.set(0,2,1.0);



                  // Best Fit Transform
                  CommonOps_DDRM.multTransA(boxCentroidSubtractedPoints, envCentroidSubtractedPoints, H);
//                  CommonOps_DDRM.multTransA(testObjPoints, testEnvPoints, H);
//                  CommonOps_DDRM.transpose(H);

                  svdSolver.decompose(H);
                  svdSolver.getU(U, true);
                  svdSolver.getV(V, true);

                  SimpleSVD svdSimpleSolver = new SimpleSVD(H,false);
                  DMatrixRMaj Utest = new DMatrixRMaj(3, 3);
                  DMatrixRMaj Vtest = new DMatrixRMaj(3, 3);
                  Utest = svdSimpleSolver.getU().getDDRM();
                  Vtest = svdSimpleSolver.getV().getDDRM();

//                  CommonOps_DDRM.transpose(Utest);
//                  CommonOps_DDRM.transpose(Vtest);
                  CommonOps_DDRM.mult(V, U, R);
//                  CommonOps_DDRM.transpose(R);

                  // check if transform wants to reflect instead of rotate and fix it
                  if (CommonOps_DDRM.det(R) < 0) {
                     Vtest.set(2, 0, -Vtest.get(2, 0));
                     Vtest.set(2, 1, -Vtest.get(2, 1));
                     Vtest.set(2, 2, -Vtest.get(2, 2));
                     CommonOps_DDRM.mult(Vtest, Utest, R);
                     System.out.println("flipped");
                  } else {
                     System.out.println("NOT");
                  }

                  CommonOps_DDRM.multTransB(R, boxCentroid, newBoxLocation);
                  CommonOps_DDRM.transpose(newBoxLocation);
//                  CommonOps_DDRM.subtract(envCentroid, newBoxLocation, boxTranslation);
                  CommonOps_DDRM.subtract(envCentroid, boxCentroid, boxTranslation);


                  boxTransform.getTranslation().addX(boxTranslation.get(0, 0));
                  boxTransform.getTranslation().addY(boxTranslation.get(0, 1));
                  boxTransform.getTranslation().addZ(boxTranslation.get(0, 2));
//                  boxTransform.getRotation().set(R);


                  //                  testPointInWorld.set(boxTransform.getTranslation().getX(), boxTransform.getTranslation().getY(), boxTransform.getTranslation().getZ());


                  // Render ICP point cloud
                  if (envPointCloud.size() >= 1) {
                     for (int i = 0; i < envPointCloud.size(); i++) {
                        envPointCloud.get(i).add(0, 0, .01);
                     }
//                     envPointCloudRenderer.setPointsToRender(envPointCloud, Color.GRAY);
//                     envPointCloudRenderer.updateMesh();
                  }
               }
            }
         }


         private void getBoxPoints(){
            boxInWorldPoints.clear();
            for (int i = 0; i < boxModelPoints.size(); i++)
            {
               Point3D32 testPoint = boxInWorldPoints.add();
               FramePoint3D testTwo = boxModelPoints.get(i);
               testTwo.add(1.49,0.995,0.51);
               testTwo.changeFrame(ReferenceFrame.getWorldFrame());
               testPoint.set(testTwo);
               testTwo.changeFrame(boxReferenceFrame);
            }
         }

         // Render ICP Panel
         private void renderImGuiICPWidgets()
         {
            DecimalFormat df = new DecimalFormat("#.###");
            ImGui.text("Mouse x: " + df.format(pickPointInWorld.getX()) + " y: " + df.format(pickPointInWorld.getY()) + " z: " + df.format(pickPointInWorld.getZ()));
            ImGui.text("Translation x: " + df.format(boxTransform.getTranslation().getX()) + " y: " + df.format(boxTransform.getTranslation().getY()) + " z: " + df.format(boxTransform.getTranslation().getZ()));
            ImGui.text("Rotation Matrix");
            ImGui.text(df.format(boxTransform.getRotation().getM00()) + "  " + df.format(boxTransform.getRotation().getM01()) + "  " + df.format(boxTransform.getRotation().getM02()));
            ImGui.text(df.format(boxTransform.getRotation().getM10()) + "  " + df.format(boxTransform.getRotation().getM11()) + "  " + df.format(boxTransform.getRotation().getM12()));
            ImGui.text(df.format(boxTransform.getRotation().getM20()) + "  " + df.format(boxTransform.getRotation().getM21()) + "  " + df.format(boxTransform.getRotation().getM22()));
            ImGui.checkbox("Select ICP point", ICPToggle);
         }





         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());
            counter = baseUI.getRenderIndex();
            double x = 1.5;
//            double y = (Math.sin(((double)counter)*0.0125));
            double y = 1.0;
            double z = 0.5;

//            double x = (Math.sin(((double)counter)*0.005)*2.0)+2.5;
//            double y = (Math.sin(((double)counter)*0.0125)*2.0);
//            double z = (Math.sin(((double)counter)*0.0165)*.5)+1.;
//            realBoxPose.set(x,y,z,0.785,0,0);
            realBoxPose.set(x,y,z,0.0,0,0);
            testPointInWorld.set(x,y,z);
            realBox.setPoseInWorld(realBoxPose);

            int aliasedRenderedAreaWidth = (int) baseUI.getPrimary3DPanel().getRenderSizeX();
            int aliasedRenderedAreaHeight = (int) baseUI.getPrimary3DPanel().getRenderSizeY();
            ByteBuffer depthBuffer = baseUI.getPrimary3DPanel().getNormalizedDeviceCoordinateDepthDirectByteBuffer();
            if (depthBuffer != null)
            {
               if (image == null)
               {
                  image = new BytedecoImage((int) baseUI.getPrimary3DPanel().getRenderSizeX(),
                                            (int) baseUI.getPrimary3DPanel().getRenderSizeY(),
                                            opencv_core.CV_32FC1,
                                            depthBuffer);
                  mainViewDepthPanel = new RDXBytedecoImagePanel("Main view depth", (int) baseUI.getPrimary3DPanel().getRenderSizeX(),
                                                                 (int) baseUI.getPrimary3DPanel().getRenderSizeY(),
                                                                 true);
                  baseUI.getImGuiPanelManager().addPanel(mainViewDepthPanel.getImagePanel());

                  baseUI.getLayoutManager().reloadLayout();
               }

               image.resize(aliasedRenderedAreaWidth, aliasedRenderedAreaHeight, null, depthBuffer);
               mainViewDepthPanel.resize(aliasedRenderedAreaWidth, aliasedRenderedAreaHeight, null);
               mainViewDepthPanel.drawDepthImage(image.getBytedecoOpenCVMat());
            }




//            getBoxPoints();
//            ICPMainFunction();

            boxPointCloudRenderer.create(boxModelPoints.size());
            boxPointCloudRenderer.setPointsToRender(boxInWorldPoints, Color.BLUE);
            boxPointCloudRenderer.updateMesh();



            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            highLevelDepthSensorSimulator.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXIterativeClosestPointReverseDemo();
   }
}
