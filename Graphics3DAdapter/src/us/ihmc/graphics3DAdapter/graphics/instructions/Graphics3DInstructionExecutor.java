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
            Graphics3DAddModelFileInstruction graphics3DAddModelFile = (Graphics3DAddModelFileInstruction) instruction;
            doAddModelFileInstruction(graphics3DAddModelFile);

         }
         else if (instruction instanceof Graphics3DAddArcTorusInstruction)
         {
            Graphics3DAddArcTorusInstruction graphics3DAddArcTorus = (Graphics3DAddArcTorusInstruction) instruction;
            doAddArcTorusInstruction(graphics3DAddArcTorus);

         }
         else if (instruction instanceof Graphics3DAddConeInstruction)
         {
            Graphics3DAddConeInstruction graphics3DAddCone = (Graphics3DAddConeInstruction) instruction;
            doAddConeInstruction(graphics3DAddCone);
         }
         else if (instruction instanceof Graphics3DAddCylinderInstruction)
         {
            Graphics3DAddCylinderInstruction graphics3DAddCylinder = (Graphics3DAddCylinderInstruction) instruction;
            doAddCylinderInstruction(graphics3DAddCylinder);
         }
         else if (instruction instanceof Graphics3DAddCoordinateSystemInstruction)
         {
            Graphics3DAddCoordinateSystemInstruction graphics3DAddCoordinateSystem = (Graphics3DAddCoordinateSystemInstruction) instruction;
            doAddCoordinateSystemInstruction(graphics3DAddCoordinateSystem);
         }
         else if (instruction instanceof Graphics3DAddCubeInstruction)
         {
            Graphics3DAddCubeInstruction graphics3DAddCube = (Graphics3DAddCubeInstruction) instruction;
            doAddCubeInstruction(graphics3DAddCube);
         }
         else if (instruction instanceof Graphics3DAddEllipsoidInstruction)
         {
            Graphics3DAddEllipsoidInstruction graphics3DAddEllipsoid = (Graphics3DAddEllipsoidInstruction) instruction;
            doAddEllipsoidInstruction(graphics3DAddEllipsoid);
         }
         else if (instruction instanceof Graphics3DAddHemiEllipsoidInstruction)
         {
            Graphics3DAddHemiEllipsoidInstruction graphics3DAddHemiEllipsoid = (Graphics3DAddHemiEllipsoidInstruction) instruction;
            doAddHemiEllipsoidInstruction(graphics3DAddHemiEllipsoid);

         }
         else if (instruction instanceof Graphics3DAddPyramidCubeInstruction)
         {
            Graphics3DAddPyramidCubeInstruction graphics3DAddPyramidCube = (Graphics3DAddPyramidCubeInstruction) instruction;
            doAddPyramidCubeInstruction(graphics3DAddPyramidCube);
         }
         else if (instruction instanceof Graphics3DAddSphereInstruction)
         {
            Graphics3DAddSphereInstruction graphics3DAddSphere = (Graphics3DAddSphereInstruction) instruction;
            doAddSphereInstruction(graphics3DAddSphere);
         }
         else if (instruction instanceof GraphicsAddVRMLFileInstruction)
         {
            GraphicsAddVRMLFileInstruction graphics3DAddVRMLFile = (GraphicsAddVRMLFileInstruction) instruction;
            doAddVRMLFileInstruction(graphics3DAddVRMLFile);
         }
         else if (instruction instanceof Graphics3DAddWedgeInstruction)
         {
            Graphics3DAddWedgeInstruction graphics3DAddWedge = (Graphics3DAddWedgeInstruction) instruction;
            doAddWedgeInstruction(graphics3DAddWedge);
         }
         else if (instruction instanceof Graphics3DAddTruncatedConeInstruction)
         {
            Graphics3DAddTruncatedConeInstruction graphics3DAddTruncatedCone = (Graphics3DAddTruncatedConeInstruction) instruction;
            doAddTruncatedConeInstruction(graphics3DAddTruncatedCone);
         }
         else if (instruction instanceof Graphics3DAddPolygonInstruction)
         {
            Graphics3DAddPolygonInstruction graphics3DAddPolygonDouble = (Graphics3DAddPolygonInstruction) instruction;
            doAddPolygonDoubleInstruction(graphics3DAddPolygonDouble);
         }
         else if (instruction instanceof Graphics3DAddExtrudedPolygonInstruction)
         {
            Graphics3DAddExtrudedPolygonInstruction graphics3DAddExtrudedPolygon = (Graphics3DAddExtrudedPolygonInstruction) instruction;
            doAddExtrudedPolygonInstruction(graphics3DAddExtrudedPolygon);
         }
         else if (instruction instanceof Graphics3DIdentityInstruction)
         {
            doIdentityInstruction();
         }
         else if (instruction instanceof Graphics3DRotateInstruction)
         {
            Graphics3DRotateInstruction graphics3DRotate = (Graphics3DRotateInstruction) instruction;
            doRotateInstruction(graphics3DRotate);
         }
         else if (instruction instanceof Graphics3DRotateMatrixInstruction)
         {
            Graphics3DRotateMatrixInstruction graphics3DRotateMatrix = (Graphics3DRotateMatrixInstruction) instruction;
            doRotateMatrixInstruction(graphics3DRotateMatrix);
         }
         else if (instruction instanceof Graphics3DScaleInstruction)
         {
            Graphics3DScaleInstruction graphics3DScale = (Graphics3DScaleInstruction) instruction;
            doScaleInstruction(graphics3DScale);
         }
         else if (instruction instanceof Graphics3DTranslateInstruction)
         {
            Graphics3DTranslateInstruction graphics3DTranslate = (Graphics3DTranslateInstruction) instruction;
            doTranslateInstruction(graphics3DTranslate);
         }
         else if (instruction instanceof Graphics3DAddTextInstruction)
         {
            Graphics3DAddTextInstruction graphics3DAddText = (Graphics3DAddTextInstruction) instruction;
            doAddTextInstruction(graphics3DAddText);
         }
         else if (instruction instanceof Graphics3DAddTeaPotInstruction)
         {
            Graphics3DAddTeaPotInstruction graphics3DAddTeaPot = (Graphics3DAddTeaPotInstruction) instruction;
            doAddTeaPotInstruction(graphics3DAddTeaPot);
         }
         else if (instruction instanceof Graphics3DAddHeightMapInstruction)
         {
            Graphics3DAddHeightMapInstruction graphics3DAddHeightMap = (Graphics3DAddHeightMapInstruction) instruction;
            doAddHeightMapInstruction(graphics3DAddHeightMap);
         }
         else if (instruction instanceof Graphics3DAddMeshDataInstruction)
         {
            Graphics3DAddMeshDataInstruction graphics3DAddMeshData = (Graphics3DAddMeshDataInstruction) instruction;
            doAddMeshDataInstruction(graphics3DAddMeshData);
         }
         else
         {
            System.err.println("Unknown graphics3DDefinition: " + instruction.getClass().getSimpleName());
         }

      }

   }

   protected abstract void doAddMeshDataInstruction(Graphics3DAddMeshDataInstruction graphics3DAddMeshData);

   protected abstract void doAddTeaPotInstruction(Graphics3DAddTeaPotInstruction graphics3DAddTeaPot);

   protected abstract void doAddHeightMapInstruction(Graphics3DAddHeightMapInstruction graphics3DAddHeightMap);

   protected abstract void doAddTextInstruction(Graphics3DAddTextInstruction graphics3DAddText);

   protected abstract void doAddArcTorusInstruction(Graphics3DAddArcTorusInstruction graphics3DAddArcTorus);

   protected abstract void doAddModelFileInstruction(Graphics3DAddModelFileInstruction graphics3DAddModelFile);

   protected abstract void doAddConeInstruction(Graphics3DAddConeInstruction graphics3DAddCone);

   protected abstract void doAddCylinderInstruction(Graphics3DAddCylinderInstruction graphics3DAddCylinder);

   protected abstract void doAddCoordinateSystemInstruction(Graphics3DAddCoordinateSystemInstruction graphics3DAddCoordinateSystem);

   protected abstract void doAddCubeInstruction(Graphics3DAddCubeInstruction graphics3DAddCube);

   protected abstract void doAddEllipsoidInstruction(Graphics3DAddEllipsoidInstruction graphics3DAddEllipsoid);

   protected abstract void doAddHemiEllipsoidInstruction(Graphics3DAddHemiEllipsoidInstruction graphics3DAddHemiEllipsoid);

   protected abstract void doAddPolygonDoubleInstruction(Graphics3DAddPolygonInstruction graphics3DAddPolygonDouble);

   protected abstract void doAddExtrudedPolygonInstruction(Graphics3DAddExtrudedPolygonInstruction graphics3DAddExtrudedPolygon);

   protected abstract void doAddPyramidCubeInstruction(Graphics3DAddPyramidCubeInstruction graphics3DAddPyramidCube);

   protected abstract void doAddSphereInstruction(Graphics3DAddSphereInstruction graphics3DAddSphere);

   protected abstract void doAddVRMLFileInstruction(GraphicsAddVRMLFileInstruction graphics3DAddVRMLFile);

   protected abstract void doAddWedgeInstruction(Graphics3DAddWedgeInstruction graphics3DAddWedge);

   protected abstract void doAddTruncatedConeInstruction(Graphics3DAddTruncatedConeInstruction graphics3DAddTruncatedCone);

   protected abstract void doIdentityInstruction();

   protected abstract void doRotateInstruction(Graphics3DRotateInstruction graphics3DRotate);

   protected abstract void doRotateMatrixInstruction(Graphics3DRotateMatrixInstruction graphics3DRotateMatrix);

   protected abstract void doScaleInstruction(Graphics3DScaleInstruction graphics3DScale);

   protected abstract void doTranslateInstruction(Graphics3DTranslateInstruction graphics3DTranslate);

}
