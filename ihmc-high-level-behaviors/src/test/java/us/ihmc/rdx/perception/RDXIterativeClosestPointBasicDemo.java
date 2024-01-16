package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.io.BufferedReader;
import java.io.FileReader;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class RDXIterativeClosestPointBasicDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   Random random = new Random(2023);

   private final ImBoolean icpGuiOnOffToggle = new ImBoolean(true);
   private final ImBoolean icpGuiResetEnv = new ImBoolean(false);
   private final ImBoolean icpGuiAutoMoveEnv = new ImBoolean(true);
   private final float[] icpGuiEnvSetPostionX = {0.0f};
   private final float[] icpGuiEnvSetPostionY = {0.0f};
   private final float[] icpGuiEnvSetPostionZ = {0.0f};
   private final float[] icpGuiEnvSetRotationX = {0.0f};
   private final float[] icpGuiEnvSetRotationY = {0.0f};
   private final float[] icpGuiEnvSetRotationZ = {0.0f};
   private final float[] icpGuiEnvAutoMoveSpeed = {1.0f};
   private final float[] icpGuiEnvGaussianNoiseScale = {0.0f};
   private final int[] icpGuiNumICPIterations = {1};
   private double icpGuiICPRunTimeInSeconds = 0;

   private final RDXPointCloudRenderer envPointCloudRenderer = new RDXPointCloudRenderer();
   private final RDXPointCloudRenderer objectPointCloudRenderer = new RDXPointCloudRenderer();

   private final RecyclingArrayList<Point3D32> envPointCloud = new RecyclingArrayList<>(Point3D32::new);
   private final RecyclingArrayList<FramePoint3D> objectModelPointCloud = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> envModelPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<Point3D32> objectInWorldPoints = new RecyclingArrayList<>(Point3D32::new);

   private final RigidBodyTransform objectTransform = new RigidBodyTransform();
   private final ReferenceFrame objectReferenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), objectTransform);
   private final FramePoint3D pointA = new FramePoint3D(objectReferenceFrame);
   private final RigidBodyTransform envTransform = new RigidBodyTransform();
   private final ReferenceFrame envReferenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), envTransform);
   private final FramePoint3D pointB = new FramePoint3D(envReferenceFrame);

   private final int envSize = 1000;

   private final DMatrixRMaj objectCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj envCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj objectCentroidSubtractedPoints = new DMatrixRMaj(envSize, 3);
   private final DMatrixRMaj envCentroidSubtractedPoints= new DMatrixRMaj(envSize, 3);
   private final DMatrixRMaj envToObjectCorrespondencePoints = new DMatrixRMaj(envSize, 3);
   private final DMatrixRMaj zeroMatrixPoint = new DMatrixRMaj(1,3);

   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);



   public RDXIterativeClosestPointBasicDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            // Create base GUI
            baseUI.create();
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-3.0, 0.0, 1);

            // Create point cloud renderer for icp points
            baseUI.getPrimaryScene().addRenderableProvider(envPointCloudRenderer, RDXSceneLevel.VIRTUAL);
            envPointCloudRenderer.create(envSize);

            // Create Shape
            createICPPointCloudBox();
//            createICPPointCloudCone();
//            createICPPointCloudCylinder();
//            createICPPointCloudCSV();

            // Create Renderables for Object
            baseUI.getPrimaryScene().addRenderableProvider(objectPointCloudRenderer, RDXSceneLevel.VIRTUAL);

            // Create Transform and Renderables for Environment
            RDXInteractableReferenceFrame interactableReferenceFrameEnv = new RDXInteractableReferenceFrame();
            interactableReferenceFrameEnv.create(envReferenceFrame, envTransform, 1.0, baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableReferenceFrameEnv::process3DViewInput);

            // Create ICP panel
            baseUI.getImGuiPanelManager().addPanel("Iterative Closest Point", this::renderImGuiICPWidgets);

            // create extra 3d panel
            RDX3DPanel panel3D = new RDX3DPanel("3D View 2", true);
            baseUI.add3DPanel(panel3D);

            getObjectPoints();
          }


          // Create Box Object
         private void createICPPointCloudBox(){
            Random random = new Random(0);
            float boxSize = 0.35f;
            float boxHalfSize = boxSize/2;
            float halfBoxWidth = (float) RigidBodySceneObjectDefinitions.BOX_WIDTH/2.0f;
            float halfBoxDepth = (float)RigidBodySceneObjectDefinitions.BOX_DEPTH/2.0f;
            float halfBoxHeight = (float)RigidBodySceneObjectDefinitions.BOX_HEIGHT/2.0f;
            for (int i = 0; i < envSize; i++) {
               int j = random.nextInt(6);
               float x =(float)random.nextDouble(-halfBoxDepth, halfBoxDepth);
               float y =(float)random.nextDouble(-halfBoxWidth, halfBoxWidth);
               float z =(float)random.nextDouble(-halfBoxHeight, halfBoxHeight);
               if (j==0 | j==1) {x = (-(j&1)*halfBoxDepth*2.0f)+halfBoxDepth;}
               if (j==2 | j==3) {y = (-(j&1)*halfBoxWidth*2.0f)+halfBoxWidth;}
               if (j==4 | j==5) {z = (-(j&1)*halfBoxHeight*2.0f)+halfBoxHeight;}
               pointA.setToZero(objectReferenceFrame);
               pointA.set(x,y,z);
               FramePoint3D worldFramePoint = objectModelPointCloud.add();
               worldFramePoint.setIncludingFrame(pointA);
            }
            for (int i = 0; i < envSize; i++) {
               int j = random.nextInt(3);
               j=j*2;
               float x =(float)random.nextDouble(-halfBoxDepth, halfBoxDepth);
               float y =(float)random.nextDouble(-halfBoxWidth, halfBoxWidth);
               float z =(float)random.nextDouble(-halfBoxHeight, halfBoxHeight);
               if (j==0 | j==1) {x = (-(j&1)*halfBoxDepth*2.0f)+halfBoxDepth;}
               if (j==2 | j==3) {y = (-(j&1)*halfBoxWidth*2.0f)+halfBoxWidth;}
               if (j==4 | j==5) {z = (-(j&1)*halfBoxHeight*2.0f)+halfBoxHeight;}
               pointB.setToZero(envReferenceFrame);
               pointB.set(x,y,z);
               FramePoint3D envFramePoint = envModelPoints.add();
               envFramePoint.setIncludingFrame(pointB);
            }
         }

         // Create Box Object
         private void createICPPointCloudCone(){
            Random random = new Random(0);
            float coneLength = 0.4f;
            float coneRadius = 0.2f;
            for (int i = 0; i < envSize; i++) {
               float z = (float)random.nextDouble(0, coneLength);
               double phi = random.nextDouble(0, 2*Math.PI);
               float x = (float)Math.cos(phi)*z*(coneRadius/coneLength);
               float y =(float)Math.sin(phi)*z*(coneRadius/coneLength);
               pointA.setToZero(objectReferenceFrame);
               pointA.set(x,y,z);
               FramePoint3D worldFramePoint = objectModelPointCloud.add();
               worldFramePoint.setIncludingFrame(pointA);
            }
            for (int i = 0; i < envSize; i++) {
               float z = (float)random.nextDouble(0, coneLength);
               double phi = random.nextDouble(0, Math.PI);
               float x = (float)Math.cos(phi)*z*(coneRadius/coneLength);
               float y =(float)Math.sin(phi)*z*(coneRadius/coneLength);
               pointB.setToZero(envReferenceFrame);
               pointB.set(x,y,z);
               FramePoint3D envFramePoint = envModelPoints.add();
               envFramePoint.setIncludingFrame(pointB);
            }
         }


         // Create Box Cylinder
         private void createICPPointCloudCylinder(){
            Random random = new Random(0);
            float CylinderLength = 1.0f;
            float CylinderRadius = 0.2f;
            for (int i = 0; i < envSize; i++) {
               int j = random.nextInt(6);
               float z = (float)random.nextDouble(0, CylinderLength);
               float r = CylinderRadius;
               if (j==0) {z = 0; r = (float)random.nextDouble(0, CylinderRadius);}
               if (j==1) {z = CylinderLength; r = (float)random.nextDouble(0, CylinderRadius);}
               double phi = random.nextDouble(0, 2*Math.PI);
               float x = (float)Math.cos(phi)*r;
               float y =(float)Math.sin(phi)*r;
               pointA.setToZero(objectReferenceFrame);
               pointA.set(x,y,z);
               FramePoint3D worldFramePoint = objectModelPointCloud.add();
               worldFramePoint.setIncludingFrame(pointA);
            }
            for (int i = 0; i < envSize; i++) {
               int j = random.nextInt(6);
               float z = (float)random.nextDouble(0, CylinderLength);
               float r = CylinderRadius;
               if (j==0) {z = 0; r = (float)random.nextDouble(0, CylinderRadius);}
               if (j==1) {z = CylinderLength; r = (float)random.nextDouble(0, CylinderRadius);}
               double phi = random.nextDouble(0, 2*Math.PI);
               float x = (float)Math.cos(phi)*r;
               float y =(float)Math.sin(phi)*r;
               pointB.setToZero(envReferenceFrame);
               pointB.set(x,y,z);
               FramePoint3D envFramePoint = envModelPoints.add();
               envFramePoint.setIncludingFrame(pointB);
            }
         }


         // Create Object from CSV
//         Box.csv
//         mug_modified_accurate.csv
         private void createICPPointCloudCSV() {
            Random random = new Random(0);
            List<String[]> rowList = new ArrayList<String[]>();
            try (BufferedReader br = new BufferedReader(new FileReader("/home/gclark/Downloads/mug_modified_accurate.csv"))) {
               String line;
               while ((line = br.readLine()) != null) {
                  String[] lineItems = line.split(",");
                  rowList.add(lineItems);
               }
               br.close();
            }
            catch (Exception e) {
            // Handle any I/O problems
            }

            for (int i = 0; i < envSize; i++) {
               int j = random.nextInt(rowList.size()-1);
               String[] row = rowList.get(j+1);
               float x = Float.parseFloat(row[0]);
               float y = Float.parseFloat(row[1]);
               float z = Float.parseFloat(row[2]);
               pointA.setToZero(objectReferenceFrame);
               pointA.set(x,y,z);
               FramePoint3D worldFramePoint = objectModelPointCloud.add();
               worldFramePoint.setIncludingFrame(pointA);
            }
            for (int i = 0; i < envSize; i++) {
               int j = random.nextInt(rowList.size()-1);
               String[] row = rowList.get(j+1);
               float x = Float.parseFloat(row[0]);
               float y = Float.parseFloat(row[1]);
               float z = Float.parseFloat(row[2]);
               pointB.setToZero(envReferenceFrame);
               pointB.set(x,y,z);
               FramePoint3D envFramePoint = envModelPoints.add();
               envFramePoint.setIncludingFrame(pointB);
            }

            }

         // Render ICP Panel
         private void renderImGuiICPWidgets()
         {
            DecimalFormat tf = new DecimalFormat("#.#########");
            DecimalFormat df = new DecimalFormat("#.###");
            ImGui.text("ICP Time: " + tf.format(icpGuiICPRunTimeInSeconds));
            ImGui.text(" ");
            ImGui.text("Env Centroid: " + df.format(envCentroid.get(0, 0)) + " y: " + df.format(envCentroid.get(0, 1)) + " z: " + df.format(envCentroid.get(0, 2)));
            ImGui.text("Obj Centroid: " + df.format(objectCentroid.get(0, 0)) + " y: " + df.format(objectCentroid.get(0, 1)) + " z: " + df.format(objectCentroid.get(0, 2)));
            ImGui.text("diff Centroid: " + df.format(envCentroid.get(0, 0)-objectCentroid.get(0, 0)) + " y: " + df.format(envCentroid.get(0, 1)-objectCentroid.get(0, 1)) + " z: " + df.format(envCentroid.get(0, 2)-objectCentroid.get(0, 2)));
            ImGui.text(" ");
            ImGui.sliderFloat("X Position", icpGuiEnvSetPostionX, -2.0f, 2.0f);
            ImGui.sliderFloat("Y Position", icpGuiEnvSetPostionY, -2.0f, 2.0f);
            ImGui.sliderFloat("Z Position", icpGuiEnvSetPostionZ, -2.0f, 2.0f);
            ImGui.text(" ");
            ImGui.sliderFloat("X Rotation", icpGuiEnvSetRotationX, -3.14f, 3.14f);
            ImGui.sliderFloat("Y Rotation", icpGuiEnvSetRotationY, -3.14f, 3.14f);
            ImGui.sliderFloat("Z Rotation", icpGuiEnvSetRotationZ, -3.14f, 3.14f);
            ImGui.text(" ");
            ImGui.sliderFloat("Env Noise", icpGuiEnvGaussianNoiseScale, 0.0f, 0.2f);
            ImGui.text(" ");
            ImGui.text(" ");
            ImGui.checkbox("ICP On", icpGuiOnOffToggle);
            ImGui.checkbox("Reset env", icpGuiResetEnv);
            ImGui.checkbox("Auto Move", icpGuiAutoMoveEnv);
            ImGui.sliderFloat("Move Speed", icpGuiEnvAutoMoveSpeed, 0.0f, 10.0f);
            ImGui.sliderInt("# iterations / tick", icpGuiNumICPIterations,1, 10);
         }


         private void ICPMainFunction() {
            getEnvPoints();
            float cuttoffRange = .5f;
            if (icpGuiOnOffToggle.get()) {

               // START ICP CALCS
               // Calc nearest neighbor env to object
               if (envPointCloud.size() >= envSize) {
                  envPointCloud.shuffle(random);
                  for (int i = 0; i < envSize; i++) {
                     Point3D32 envPoint = envPointCloud.get(i);
                     float minDistance = Float.MAX_VALUE;
                     int minIndice = Integer.MAX_VALUE;

                     for (int j = 0; j < objectModelPointCloud.size(); j++) {
                        float distance = (float) envPoint.distance(objectInWorldPoints.get(j));
                        if (distance <= minDistance) {
                           minDistance = distance;
                           minIndice = j;
                        }
                     }
                     envToObjectCorrespondencePoints.set(i, 0, objectInWorldPoints.get(minIndice).getX());
                     envToObjectCorrespondencePoints.set(i, 1, objectInWorldPoints.get(minIndice).getY());
                     envToObjectCorrespondencePoints.set(i, 2, objectInWorldPoints.get(minIndice).getZ());
                  }

                  // Calculate obj centroid
                  objectCentroid.set(zeroMatrixPoint);
                  for (int i = 0; i < envSize; i++) {
                     objectCentroid.add(0, 0, envToObjectCorrespondencePoints.get(i, 0));
                     objectCentroid.add(0, 1, envToObjectCorrespondencePoints.get(i, 1));
                     objectCentroid.add(0, 2, envToObjectCorrespondencePoints.get(i, 2));
                  }
                  objectCentroid.set(0, 0, objectCentroid.get(0, 0) / envToObjectCorrespondencePoints.numRows);
                  objectCentroid.set(0, 1, objectCentroid.get(0, 1) / envToObjectCorrespondencePoints.numRows);
                  objectCentroid.set(0, 2, objectCentroid.get(0, 2) / envToObjectCorrespondencePoints.numRows);

                  for (int i = 0; i < envSize; i++) {
                     objectCentroidSubtractedPoints.set(i, 0, envToObjectCorrespondencePoints.get(i, 0) - objectCentroid.get(0, 0));
                     objectCentroidSubtractedPoints.set(i, 1, envToObjectCorrespondencePoints.get(i, 1) - objectCentroid.get(0, 1));
                     objectCentroidSubtractedPoints.set(i, 2, envToObjectCorrespondencePoints.get(i, 2) - objectCentroid.get(0, 2));
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
                  // Subtract env centroid from env point cloud
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
                  DMatrixRMaj objAdjustedLocation = new DMatrixRMaj(3, 1);
                  DMatrixRMaj objTranslation = new DMatrixRMaj(1, 3);
                  DMatrixRMaj T = new DMatrixRMaj(4, 4);
                  CommonOps_DDRM.setIdentity(T);
                  DMatrixRMaj interimPoint = new DMatrixRMaj(1, 3);
                  DMatrixRMaj movedPoint = new DMatrixRMaj(1, 3);

                  // Solve for Best Fit Transformation
                  CommonOps_DDRM.multTransA(objectCentroidSubtractedPoints, envCentroidSubtractedPoints, H);
                  svdSolver.decompose(H);
                  svdSolver.getU(U, false);
                  svdSolver.getV(V, true);
                  CommonOps_DDRM.multTransAB(V, U, R);

                  // Calculate object translation
                  CommonOps_DDRM.multTransB(R, objectCentroid, objAdjustedLocation);
                  CommonOps_DDRM.transpose(objAdjustedLocation);
                  CommonOps_DDRM.subtract(envCentroid, objAdjustedLocation, objTranslation);


                  // Rotate and translate object points
                  for (Point3D32 objectInWorldPoint : objectInWorldPoints) {
                     interimPoint.set(new double[][]{{objectInWorldPoint.getX()}, {objectInWorldPoint.getY()}, {objectInWorldPoint.getZ()}});
                     CommonOps_DDRM.mult(R, interimPoint, movedPoint);
                     objectInWorldPoint.set(movedPoint.get(0) + objTranslation.get(0), movedPoint.get(1) + objTranslation.get(1), movedPoint.get(2) + objTranslation.get(2));
                  }
               }
            }
         }


         private void getObjectPoints(){
            objectInWorldPoints.clear();
            for (FramePoint3D objectModelPoint : objectModelPointCloud) {
               Point3D32 testPoint = objectInWorldPoints.add();
               objectModelPoint.changeFrame(ReferenceFrame.getWorldFrame());
               testPoint.set(objectModelPoint);
               objectModelPoint.changeFrame(objectReferenceFrame);
            }
         }


         private void getEnvPoints(){
            envPointCloud.clear();
            for (FramePoint3D envModelPoint : envModelPoints) {
               Point3D32 testPoint = envPointCloud.add();
               envModelPoint.changeFrame(ReferenceFrame.getWorldFrame());
               double xNoise = icpGuiEnvGaussianNoiseScale[0] * (random.nextDouble() - 0.5);
               double yNoise = icpGuiEnvGaussianNoiseScale[0] * (random.nextDouble() - 0.5);
               double zNoise = icpGuiEnvGaussianNoiseScale[0] * (random.nextDouble() - 0.5);
               testPoint.set(envModelPoint.getX() + xNoise, envModelPoint.getY() + yNoise, envModelPoint.getZ() + zNoise);
               envModelPoint.changeFrame(envReferenceFrame);
            }
         }


         private void resetEnv()
         {
            if (icpGuiResetEnv.get()) {
               icpGuiEnvSetPostionX[0]=0.0f;
               icpGuiEnvSetPostionY[0]=0.0f;
               icpGuiEnvSetPostionZ[0]=0.0f;
               icpGuiEnvSetRotationX[0]=0.0f;
               icpGuiEnvSetRotationY[0]=0.0f;
               icpGuiEnvSetRotationZ[0]=0.0f;
               icpGuiResetEnv.set(false);
            }
         }


         private void calculateICPTime(long time1, long time2){
            long timeDiffNanos = time2-time1;
            icpGuiICPRunTimeInSeconds = (timeDiffNanos/1e9)/icpGuiNumICPIterations[0];
         }


         private void moveEnvObject(){
            long counter = baseUI.getRenderIndex();
            if (icpGuiAutoMoveEnv.get()) {
               double x = (Math.sin(((double)counter)*0.001*icpGuiEnvAutoMoveSpeed[0])*2)+2.5;
               double y = (Math.sin(((double)counter)*0.0033*icpGuiEnvAutoMoveSpeed[0])*1);
               double z = (Math.sin(((double)counter)*0.004*icpGuiEnvAutoMoveSpeed[0])*.5)+.5;
               double roll = (Math.sin(((double)counter)*0.002*icpGuiEnvAutoMoveSpeed[0])*3.14159);
               double pitch= (Math.sin(((double)counter)*0.0023*icpGuiEnvAutoMoveSpeed[0])*3.14159);
               double yaw =  (Math.sin(((double)counter)*0.00275*icpGuiEnvAutoMoveSpeed[0])*3.14159);
               envTransform.getTranslation().set(x,y,z);
               envTransform.getRotation().setEuler(roll,pitch,yaw);
            }
            else {
               envTransform.getTranslation().set(icpGuiEnvSetPostionX[0],icpGuiEnvSetPostionY[0],icpGuiEnvSetPostionZ[0]);
               envTransform.getRotation().setEuler(icpGuiEnvSetRotationX[0],icpGuiEnvSetRotationY[0],icpGuiEnvSetRotationZ[0]);
            }
         }


         private void updateGUIPointClouds(){
            envPointCloudRenderer.setPointsToRender(envPointCloud, Color.GRAY);
            envPointCloudRenderer.updateMesh();
            objectPointCloudRenderer.create(objectModelPointCloud.size());
            objectPointCloudRenderer.setPointsToRender(objectInWorldPoints, Color.BLUE);
            objectPointCloudRenderer.updateMesh();
         }



         @Override
         public void render()
         {
            resetEnv();
            moveEnvObject();

            // Run ICP function and time
            long startTimeNanos = System.nanoTime();
            for (int k = 0; k < icpGuiNumICPIterations[0]; k++) {
               ICPMainFunction();
            }
            long stopTimeNanos = System.nanoTime();
            calculateICPTime(startTimeNanos, stopTimeNanos);

            updateGUIPointClouds();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXIterativeClosestPointBasicDemo();
   }
}
