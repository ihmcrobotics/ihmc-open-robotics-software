package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.compress.archivers.zip.ScatterStatistics;
import org.apache.commons.lang3.mutable.MutableFloat;
import org.apache.commons.lang3.mutable.MutableInt;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.ejml.simple.SimpleMatrix;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.objects.RDXArUcoBoxObject;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.robotics.linearAlgebra.ConfigurableSolvePseudoInverseSVD;
import us.ihmc.robotics.quadTree.QuadTreeForGroundPoint;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class RDXIterativeClosestPointDemo
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
   private DMatrixRMaj envCorrespondencePoints;
   private DMatrixRMaj zeroMatrixPoint = new DMatrixRMaj(1,3);

   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);






   public RDXIterativeClosestPointDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            // Create base GUI
            baseUI.create();

            // Create Environment and render panels
            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("HarderTerrain.json");

            // Place sensor in environment
            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            // Adjust sensor position to better see the box
            sensorPoseGizmo.getTransformToParent().appendTranslation(.2, -0.5, 2.);
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
//            for (int i = 0; i < 8; i++) {
//               float x = (-((i >> 0)&1)*boxSize)+boxHalfSize;
//               float y = (-((i >> 1)&1)*boxSize)+boxHalfSize;
//               float z = (-((i >> 2)&1)*boxSize)+boxHalfSize;
//               pointA.setToZero(boxReferenceFrame);
//               pointA.set(x,y,z);
//               FramePoint3D worldFramePoint = boxModelPoints.add();
//               worldFramePoint.setIncludingFrame(pointA);
//            }
//            for (int i = 0; i < 6; i++) {
//               float x =0f;float y =0f;float z =0f;
//               if (i==0 | i==1) {x = (-((i >> 0)&1)*boxSize)+boxHalfSize;}
//               if (i==2 | i==3) {y = (-((i >> 0)&1)*boxSize)+boxHalfSize;}
//               if (i==4 | i==5) {z = (-((i >> 0)&1)*boxSize)+boxHalfSize;}
//               pointA.setToZero(boxReferenceFrame);
//               pointA.set(x,y,z);
//               FramePoint3D worldFramePoint = boxModelPoints.add();
//               worldFramePoint.setIncludingFrame(pointA);
//            }

            Random random = new Random(0);
            random.nextDouble(-boxHalfSize, boxHalfSize);

            for (int i = 0; i < 6; i++) {
               for (int j = 0; j < 100; j++) {
                  float x =(float)random.nextDouble(-boxHalfSize, boxHalfSize);
                  float y =(float)random.nextDouble(-boxHalfSize, boxHalfSize);
                  float z =(float)random.nextDouble(-boxHalfSize, boxHalfSize);
                  if (i==0 | i==1) {x = (-((i >> 0)&1)*boxSize)+boxHalfSize;}
                  if (i==2 | i==3) {y = (-((i >> 0)&1)*boxSize)+boxHalfSize;}
                  if (i==4 | i==5) {z = (-((i >> 0)&1)*boxSize)+boxHalfSize;}
                  pointA.setToZero(boxReferenceFrame);
                  pointA.set(x,y,z);
                  FramePoint3D worldFramePoint = boxModelPoints.add();
                  worldFramePoint.setIncludingFrame(pointA);
               }
            }










            RDXInteractableReferenceFrame interactableReferenceFrame = new RDXInteractableReferenceFrame();
            interactableReferenceFrame.create(boxReferenceFrame, boxTransform, 1.0, baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableReferenceFrame::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableReferenceFrame::process3DViewInput);
//            baseUI.getPrimaryScene().addRenderableProvider(interactableReferenceFrame::getVirtualRenderables);
            baseUI.getPrimaryScene().addRenderableProvider(boxPointCloudRenderer::getRenderables, RDXSceneLevel.VIRTUAL);

            // Pick 3D point when collision is selected
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::ICPMainFunction);

            // Create ICP panel
            baseUI.getImGuiPanelManager().addPanel("Iterative Closest Point", this::renderImGuiICPWidgets);

            // create extra 3d panel
            RDX3DPanel panel3D = new RDX3DPanel("3D View 2", true);
            baseUI.add3DPanel(panel3D);

            // Initialize ICP distances and indices
            for (int i = 0; i < boxModelPoints.size(); i++) {
               ICPDistances.add(Float.MAX_VALUE);
               ICPIndices.add(0);
            }

            boxCentroid  = new DMatrixRMaj(1, 3);
            envCentroid  = new DMatrixRMaj(1, 3);
            boxCentroidSubtractedPoints = new DMatrixRMaj(boxModelPoints.size(), 3);
            envCorrespondencePoints = new DMatrixRMaj(boxModelPoints.size(), 3);
            envCentroidSubtractedPoints = new DMatrixRMaj(boxModelPoints.size(), 3);
          }



         private void ICPMainFunction(us.ihmc.rdx.input.ImGui3DViewInput input) {
            float cuttoffRange = .4f;
            pickPointInWorld = input.getPickPointInWorld();
            LibGDXTools.toLibGDX(pickPointInWorld, mousePickSphere.transform);
            envPointCloud.clear();

            if (ICPToggle.get()) {
               List<Point3D> depthData =  highLevelDepthSensorSimulator.getPointCloud();

               // Seclect env point cloud and filter based on distance
               for (int i = 0; i < depthData.size(); i++) {
                  Point3D envPoint = depthData.get(i);
                  double pointDifferenceFromSelected = pickPointInWorld.distance(envPoint);
                  if (pointDifferenceFromSelected <= cuttoffRange)
                  {
                     envPointCloud.add().set(envPoint);
                  }
               }



               // START ICP CALCS
               // Calc nearest neighbor object to env
               if (envPointCloud.size() != 0) {
                  for (int i = 0; i < boxModelPoints.size(); i++) {
                     ICPDistances.set(i, Float.MAX_VALUE);
                     ICPIndices.set(i, 0);
                  }
                  for (int i = 0; i < boxInWorldPoints.size(); i++) {
                     Point3D32 boxPoint = boxInWorldPoints.get(i);
                     float minDistance = Float.MAX_VALUE;
                     int minIndice = Integer.MAX_VALUE;
                     for (int j = 0; j < envPointCloud.size(); j++) {
                        distance = (float) boxPoint.distance(envPointCloud.get(j));
                        if (distance <= minDistance) {
                           minDistance = distance;
                           minIndice = j;
                        }
                     }
                     ICPDistances.set(i, minDistance);
                     ICPIndices.set(i, minIndice);
                     envCorrespondencePoints.set(i, 0, envPointCloud.get(minIndice).getX());
                     envCorrespondencePoints.set(i, 1, envPointCloud.get(minIndice).getY());
                     envCorrespondencePoints.set(i, 2, envPointCloud.get(minIndice).getZ());
                  }


                  // Calculate data centroid and arrange in matrices
                  boxCentroid.set(zeroMatrixPoint);
                  for (int i = 0; i < boxModelPoints.size(); i++) {
                     boxCentroid.add(0, 0, boxInWorldPoints.get(i).getX());
                     boxCentroid.add(0, 1, boxInWorldPoints.get(i).getY());
                     boxCentroid.add(0, 2, boxInWorldPoints.get(i).getZ());
                  }
                  boxCentroid.set(0, 0, boxCentroid.get(0, 0) / boxModelPoints.size());
                  boxCentroid.set(0, 1, boxCentroid.get(0, 1) / boxModelPoints.size());
                  boxCentroid.set(0, 2, boxCentroid.get(0, 2) / boxModelPoints.size());

                  for (int i = 0; i < boxModelPoints.size(); i++) {
                     boxCentroidSubtractedPoints.set(i, 0, boxInWorldPoints.get(i).getX() - boxCentroid.get(0, 0));
                     boxCentroidSubtractedPoints.set(i, 1, boxInWorldPoints.get(i).getY() - boxCentroid.get(0, 1));
                     boxCentroidSubtractedPoints.set(i, 2, boxInWorldPoints.get(i).getZ() - boxCentroid.get(0, 2));
                  }

                  // Calculate env centroid
                  envCentroid.set(zeroMatrixPoint);
                  for (int i = 0; i < envPointCloud.size(); i++) {
                     envCentroid.add(0, 0, envPointCloud.get(i).getX());
                     envCentroid.add(0, 1, envPointCloud.get(i).getY());
                     envCentroid.add(0, 2, envPointCloud.get(i).getZ());
                  }
                  envCentroid.set(0, 0, envCentroid.get(0, 0) / envPointCloud.size());
                  envCentroid.set(0, 1, envCentroid.get(0, 1) / envPointCloud.size());
                  envCentroid.set(0, 2, envCentroid.get(0, 2) / envPointCloud.size());

                  for (int i = 0; i < boxModelPoints.size(); i++) {
                     envCentroidSubtractedPoints.set(i, 0, envCorrespondencePoints.get(i, 0) - envCentroid.get(0, 0));
                     envCentroidSubtractedPoints.set(i, 1, envCorrespondencePoints.get(i, 1) - envCentroid.get(0, 1));
                     envCentroidSubtractedPoints.set(i, 2, envCorrespondencePoints.get(i, 2) - envCentroid.get(0, 2));
                  }


                  // Initialize matrix variables
                  DMatrixRMaj H = new DMatrixRMaj(3, 3);
                  DMatrixRMaj U = new DMatrixRMaj(3, 3);
                  DMatrixRMaj Vt = new DMatrixRMaj(3, 3);
                  DMatrixRMaj R = new DMatrixRMaj(3, 3);
                  DMatrixRMaj newBoxLocation = new DMatrixRMaj(1, 3);
                  DMatrixRMaj boxTranslation = new DMatrixRMaj(1, 3);
                  DMatrixRMaj T = new DMatrixRMaj(4, 4);
                  CommonOps_DDRM.setIdentity(T);

                  // Best Fit Transform
                  CommonOps_DDRM.multTransA(boxCentroidSubtractedPoints, envCentroidSubtractedPoints, H);

                  svdSolver.decompose(H);
                  svdSolver.getU(U, false);
                  svdSolver.getV(Vt, true);


                  CommonOps_DDRM.transpose(U);
                  CommonOps_DDRM.transpose(Vt);
                  CommonOps_DDRM.mult(Vt, U, R);

                  // check if transform wants to reflect instead of rotate and fix it
                  if (CommonOps_DDRM.det(R) < 0) {
                     Vt.set(2,0,-Vt.get(2,0));
                     Vt.set(2,1,-Vt.get(2,1));
                     Vt.set(2,2,-Vt.get(2,2));
                     CommonOps_DDRM.mult(Vt, U, R);
                     System.out.println("flipped");
                  }
                  else{
                     System.out.println("NOT");
                  }

                  CommonOps_DDRM.multTransB(R, boxCentroid, newBoxLocation);
                  CommonOps_DDRM.transpose(newBoxLocation);
                  CommonOps_DDRM.subtract(envCentroid, newBoxLocation, boxTranslation);


                  boxTransform.getTranslation().addX(boxTranslation.get(0,0));
                  boxTransform.getTranslation().addY(boxTranslation.get(0,1));
                  boxTransform.getTranslation().addZ(boxTranslation.get(0,2));
                  boxTransform.getRotation().set(R);




                  // Render ICP point cloud
                  if (envPointCloud.size() >= 1) {
                     for (int i = 0; i < envPointCloud.size(); i++) {
                        envPointCloud.get(i).add(0,0,.01);
                     }
                     envPointCloudRenderer.setPointsToRender(envPointCloud, Color.GRAY);
                     envPointCloudRenderer.updateMesh();
                  }
               }
            }
         }



         // Render ICP Panel
         private void renderImGuiICPWidgets()
         {
            DecimalFormat df = new DecimalFormat("#.###");
            ImGui.text("Mouse x: " + df.format(pickPointInWorld.getX()) + " y: " + df.format(pickPointInWorld.getY()) + " z: " + df.format(pickPointInWorld.getZ()));
            ImGui.checkbox("Select ICP point", ICPToggle);
         }





         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());


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



            // Render Box Points
//            boxTransform.getTranslation().addX(0.001);
            boxInWorldPoints.clear();
            for (int i = 0; i < boxModelPoints.size(); i++)
            {
               Point3D32 testPoint = boxInWorldPoints.add();
               FramePoint3D testTwo = boxModelPoints.get(i);
               testTwo.changeFrame(ReferenceFrame.getWorldFrame());
               testPoint.set(testTwo);
               testTwo.changeFrame(boxReferenceFrame);
            }
            boxPointCloudRenderer.create(boxModelPoints.size());
            boxPointCloudRenderer.setPointsToRender(boxInWorldPoints, Color.WHITE);
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
      new RDXIterativeClosestPointDemo();
   }
}
