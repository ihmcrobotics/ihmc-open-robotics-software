package us.ihmc.graphics3DAdapter.graphics.instructions;

import java.util.ArrayList;



public abstract class Graphics3DInstructionExecutor
{
   public Graphics3DInstructionExecutor()
   {
      super();
   }

   protected void setUpGraphicsFromDefinition(ArrayList<Graphics3DPrimitiveInstruction> instructions)
   {
      for (Graphics3DPrimitiveInstruction instruction : instructions)
      {
         if (instruction instanceof Graphics3DAddModelFileInstruction)
         {
            Graphics3DAddModelFileInstruction linkGraphicsAddModelFile = (Graphics3DAddModelFileInstruction) instruction;
            doLinkGraphicsAddModelFileInstruction(linkGraphicsAddModelFile);

         }
         else if (instruction instanceof Graphics3DAddArcTorusInstruction)
         {
            Graphics3DAddArcTorusInstruction linkGraphicsAddArcTorus = (Graphics3DAddArcTorusInstruction) instruction;
            doLinkGraphicsAddArcTorusInstruction(linkGraphicsAddArcTorus);

         }
         else if (instruction instanceof Graphics3DAddConeInstruction)
         {
            Graphics3DAddConeInstruction linkGraphicsAddCone = (Graphics3DAddConeInstruction) instruction;
            doLinkGraphicsAddConeInstruction(linkGraphicsAddCone);
         }
         else if (instruction instanceof Graphics3DAddCylinderInstruction)
         {
            Graphics3DAddCylinderInstruction linkGraphicsAddCylinder = (Graphics3DAddCylinderInstruction) instruction;
            doLinkGraphicsAddCylinderInstruction(linkGraphicsAddCylinder);
         }
         else if (instruction instanceof Graphics3DAddCoordinateSystemInstruction)
         {
            Graphics3DAddCoordinateSystemInstruction linkGraphicsAddCoordinateSystem = (Graphics3DAddCoordinateSystemInstruction) instruction;
            doLinkGraphicsAddCoordinateSystemInstruction(linkGraphicsAddCoordinateSystem);
         }
         else if (instruction instanceof Graphics3DAddCubeInstruction)
         {
            Graphics3DAddCubeInstruction linkGraphicsAddCube = (Graphics3DAddCubeInstruction) instruction;
            doLinkGraphicsAddCubeInstruction(linkGraphicsAddCube);
         }
         else if (instruction instanceof Graphics3DAddEllipsoidInstruction)
         {
            Graphics3DAddEllipsoidInstruction linkGraphicsAddEllipsoid = (Graphics3DAddEllipsoidInstruction) instruction;
            doLinkGraphicsAddEllipsoidInstruction(linkGraphicsAddEllipsoid);
         }
         else if (instruction instanceof Graphics3DAddHemiEllipsoidInstruction)
         {
            Graphics3DAddHemiEllipsoidInstruction linkGraphicsAddHemiEllipsoid = (Graphics3DAddHemiEllipsoidInstruction) instruction;
            doLinkGraphicsAddHemiEllipsoidInstruction(linkGraphicsAddHemiEllipsoid);

         }
         else if (instruction instanceof Graphics3DAddPyramidCubeInstruction)
         {
            Graphics3DAddPyramidCubeInstruction linkGraphicsAddPyramidCube = (Graphics3DAddPyramidCubeInstruction) instruction;
            doLinkGraphicsAddPyramidCubeInstruction(linkGraphicsAddPyramidCube);
         }
         else if (instruction instanceof Graphics3DAddSphereInstruction)
         {
            Graphics3DAddSphereInstruction linkGraphicsAddSphere = (Graphics3DAddSphereInstruction) instruction;
            doLinkGraphicsAddSphereInstruction(linkGraphicsAddSphere);
         }
         else if (instruction instanceof GraphicsAddVRMLFileInstruction)
         {
            GraphicsAddVRMLFileInstruction linkGraphicsAddVRMLFile = (GraphicsAddVRMLFileInstruction) instruction;
            doLinkGraphicsAddVRMLFileInstruction(linkGraphicsAddVRMLFile);
         }
         else if (instruction instanceof Graphics3DAddWedgeInstruction)
         {
            Graphics3DAddWedgeInstruction linkGraphicsAddWedge = (Graphics3DAddWedgeInstruction) instruction;
            doLinkGraphicsAddWedgeInstruction(linkGraphicsAddWedge);
         }
         else if (instruction instanceof Graphics3DAddTruncatedConeInstruction)
         {
            Graphics3DAddTruncatedConeInstruction linkGraphicsAddTruncatedCone = (Graphics3DAddTruncatedConeInstruction) instruction;
            doLinkGraphicsAddTruncatedConeInstruction(linkGraphicsAddTruncatedCone);
         }
         else if (instruction instanceof Graphics3DAddPolygonInstruction)
         {
            Graphics3DAddPolygonInstruction linkGraphicsAddPolygonDouble = (Graphics3DAddPolygonInstruction) instruction;
            doLinkGraphicsAddPolygonDoubleInstruction(linkGraphicsAddPolygonDouble);
         }
         else if (instruction instanceof Graphics3DAddExtrudedPolygonInstruction)
         {
            Graphics3DAddExtrudedPolygonInstruction linkGraphicsAddExtrudedPolygon = (Graphics3DAddExtrudedPolygonInstruction) instruction;
            doLinkGraphicsAddExtrudedPolygonInstruction(linkGraphicsAddExtrudedPolygon);
         }
         else if (instruction instanceof Graphics3DIdentityInstruction)
         {
            doLinkGraphicsIdentityInstruction();
         }
         else if (instruction instanceof Graphics3DRotateInstruction)
         {
            Graphics3DRotateInstruction linkGraphicsRotate = (Graphics3DRotateInstruction) instruction;
            doLinkGraphicsRotateInstruction(linkGraphicsRotate);
         }
         else if (instruction instanceof Graphics3DRotateMatrixInstruction)
         {
            Graphics3DRotateMatrixInstruction linkGraphicsRotateMatrix = (Graphics3DRotateMatrixInstruction) instruction;
            doLinkGraphicsRotateMatrixInstruction(linkGraphicsRotateMatrix);
         }
         else if (instruction instanceof Graphics3DScaleInstruction)
         {
            Graphics3DScaleInstruction linkGraphicsScale = (Graphics3DScaleInstruction) instruction;
            doLinkGraphicsScaleInstruction(linkGraphicsScale);
         }
         else if (instruction instanceof Graphics3DTranslateInstruction)
         {
            Graphics3DTranslateInstruction linkGraphicsTranslate = (Graphics3DTranslateInstruction) instruction;
            doLinkGraphicsTranslateInstruction(linkGraphicsTranslate);
         }
         else if (instruction instanceof Graphics3DAddTextInstruction)
         {
            Graphics3DAddTextInstruction linkGraphicsAddText = (Graphics3DAddTextInstruction) instruction;
            doLinkGraphicslinkGraphicsAddTextInstruction(linkGraphicsAddText);
         }
         else if (instruction instanceof Graphics3DAddTeaPotInstruction)
         {
            Graphics3DAddTeaPotInstruction linkGraphicsAddTeaPot = (Graphics3DAddTeaPotInstruction) instruction;
            doLinkGraphicsAddTeaPot(linkGraphicsAddTeaPot);
         }
         else if (instruction instanceof Graphics3DAddHeightMapInstruction)
         {
            Graphics3DAddHeightMapInstruction linkGraphicsAddHeightMap = (Graphics3DAddHeightMapInstruction) instruction;
            doLinkGraphicsAddHeightMapInstruction(linkGraphicsAddHeightMap);
         }
         else if (instruction instanceof Graphics3DAddMeshDataInstruction)
         {
            Graphics3DAddMeshDataInstruction linkGraphicsAddMeshData = (Graphics3DAddMeshDataInstruction) instruction;
            doLinkGraphicsAddMeshDataInstruction(linkGraphicsAddMeshData);
         }
         else
         {
            System.err.println("Unknown LinkGraphicsDefinition: " + instruction.getClass().getSimpleName());
         }

      }

   }

   protected abstract void doLinkGraphicsAddMeshDataInstruction(Graphics3DAddMeshDataInstruction linkGraphicsAddMeshData);

   protected abstract void doLinkGraphicsAddTeaPot(Graphics3DAddTeaPotInstruction linkGraphicsAddTeaPot);

   protected abstract void doLinkGraphicsAddHeightMapInstruction(Graphics3DAddHeightMapInstruction linkGraphicsAddHeightMap);

   protected abstract void doLinkGraphicslinkGraphicsAddTextInstruction(Graphics3DAddTextInstruction linkGraphicsAddText);

   protected abstract void doLinkGraphicsAddArcTorusInstruction(Graphics3DAddArcTorusInstruction linkGraphicsAddArcTorus);

   protected abstract void doLinkGraphicsAddModelFileInstruction(Graphics3DAddModelFileInstruction linkGraphicsAddModelFile);

   protected abstract void doLinkGraphicsAddConeInstruction(Graphics3DAddConeInstruction linkGraphicsAddCone);

   protected abstract void doLinkGraphicsAddCylinderInstruction(Graphics3DAddCylinderInstruction linkGraphicsAddCylinder);

   protected abstract void doLinkGraphicsAddCoordinateSystemInstruction(Graphics3DAddCoordinateSystemInstruction linkGraphicsAddCoordinateSystem);

   protected abstract void doLinkGraphicsAddCubeInstruction(Graphics3DAddCubeInstruction linkGraphicsAddCube);

   protected abstract void doLinkGraphicsAddEllipsoidInstruction(Graphics3DAddEllipsoidInstruction linkGraphicsAddEllipsoid);

   protected abstract void doLinkGraphicsAddHemiEllipsoidInstruction(Graphics3DAddHemiEllipsoidInstruction linkGraphicsAddHemiEllipsoid);

   protected abstract void doLinkGraphicsAddPolygonDoubleInstruction(Graphics3DAddPolygonInstruction linkGraphicsAddPolygonDouble);

   protected abstract void doLinkGraphicsAddExtrudedPolygonInstruction(Graphics3DAddExtrudedPolygonInstruction linkGraphicsAddExtrudedPolygon);

   protected abstract void doLinkGraphicsAddPyramidCubeInstruction(Graphics3DAddPyramidCubeInstruction linkGraphicsAddPyramidCube);

   protected abstract void doLinkGraphicsAddSphereInstruction(Graphics3DAddSphereInstruction linkGraphicsAddSphere);

   protected abstract void doLinkGraphicsAddVRMLFileInstruction(GraphicsAddVRMLFileInstruction linkGraphicsAddVRMLFile);

   protected abstract void doLinkGraphicsAddWedgeInstruction(Graphics3DAddWedgeInstruction linkGraphicsAddWedge);

   protected abstract void doLinkGraphicsAddTruncatedConeInstruction(Graphics3DAddTruncatedConeInstruction linkGraphicsAddTruncatedCone);

   protected abstract void doLinkGraphicsIdentityInstruction();

   protected abstract void doLinkGraphicsRotateInstruction(Graphics3DRotateInstruction linkGraphicsRotate);

   protected abstract void doLinkGraphicsRotateMatrixInstruction(Graphics3DRotateMatrixInstruction linkGraphicsRotateMatrix);

   protected abstract void doLinkGraphicsScaleInstruction(Graphics3DScaleInstruction linkGraphicsScale);

   protected abstract void doLinkGraphicsTranslateInstruction(Graphics3DTranslateInstruction linkGraphicsTranslate);

}
