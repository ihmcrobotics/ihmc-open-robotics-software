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
         else if (instruction instanceof Graphics3DAddMeshDataInstruction)
         {
            Graphics3DAddMeshDataInstruction graphics3DAddArcTorus = (Graphics3DAddMeshDataInstruction) instruction;
            doAddMeshDataInstruction(graphics3DAddArcTorus);

         }
         else if (instruction instanceof Graphics3DIdentityInstruction)
         {
            doIdentityInstruction();
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

   protected abstract void doAddModelFileInstruction(Graphics3DAddModelFileInstruction graphics3DAddModelFile);

   protected abstract void doIdentityInstruction();

   protected abstract void doRotateMatrixInstruction(Graphics3DRotateMatrixInstruction graphics3DRotateMatrix);

   protected abstract void doScaleInstruction(Graphics3DScaleInstruction graphics3DScale);

   protected abstract void doTranslateInstruction(Graphics3DTranslateInstruction graphics3DTranslate);

}
