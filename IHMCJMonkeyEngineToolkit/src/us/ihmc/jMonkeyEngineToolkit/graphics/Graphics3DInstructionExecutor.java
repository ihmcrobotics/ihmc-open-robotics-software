package us.ihmc.jMonkeyEngineToolkit.graphics;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.instructions.CubeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddExtrusionInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddHeightMapInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DIdentityInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DRotateInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DTranslateInstruction;

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
            Graphics3DAddMeshDataInstruction meshDataInstruction = (Graphics3DAddMeshDataInstruction) instruction;
            doAddMeshDataInstruction(meshDataInstruction);
         }
         else if (instruction instanceof CubeGraphics3DInstruction)
         {
            CubeGraphics3DInstruction cubeInstruction = (CubeGraphics3DInstruction) instruction;

            MeshDataHolder meshData = MeshDataGenerator.Cube(cubeInstruction.getLength(), cubeInstruction.getWidth(), cubeInstruction.getHeight(),
                                                             cubeInstruction.getCenteredInTheCenter(), cubeInstruction.getTextureFaces());
            Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, cubeInstruction.getAppearance());

            doAddMeshDataInstruction(meshDataInstruction);
         }
         else if (instruction instanceof Graphics3DIdentityInstruction)
         {
            doIdentityInstruction();
         }
         else if (instruction instanceof Graphics3DRotateInstruction)
         {
            Graphics3DRotateInstruction graphics3DRotateMatrix = (Graphics3DRotateInstruction) instruction;
            doRotateInstruction(graphics3DRotateMatrix);
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
         else if (instruction instanceof Graphics3DAddExtrusionInstruction)
         {
            Graphics3DAddExtrusionInstruction graphics3DAddExtrusion = (Graphics3DAddExtrusionInstruction) instruction;
            doAddExtrusionInstruction(graphics3DAddExtrusion);
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

   protected abstract void doAddHeightMapInstruction(Graphics3DAddHeightMapInstruction graphics3DAddHeightMap);

   protected abstract void doAddExtrusionInstruction(Graphics3DAddExtrusionInstruction graphics3DAddText);

   protected abstract void doAddModelFileInstruction(Graphics3DAddModelFileInstruction graphics3DAddModelFile);

   protected abstract void doIdentityInstruction();

   protected abstract void doRotateInstruction(Graphics3DRotateInstruction graphics3DRotateMatrix);

   protected abstract void doScaleInstruction(Graphics3DScaleInstruction graphics3DScale);

   protected abstract void doTranslateInstruction(Graphics3DTranslateInstruction graphics3DTranslate);
}
