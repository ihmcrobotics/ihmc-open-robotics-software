package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.IOException;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotController.RobotController;

public class YoWhiteBoardWriteController implements RobotController, OutputProcessor
{
   private final YoVariableRegistry registry;
   private final IntegerYoVariable ticksTillNextWrite;
   
   private final YoWhiteBoard yoWhiteBoard;
   private final int writeEveryNTicks;

   private boolean writeOnInitialize;

   public YoWhiteBoardWriteController(String name, YoWhiteBoard yoWhiteBoard, int writeEveryNTicks, boolean writeOnInitialize)
   {
      registry = new YoVariableRegistry(name + "YoWhiteBoardWriteController");
      this.yoWhiteBoard = yoWhiteBoard;
      
      if (writeEveryNTicks < 1) throw new RuntimeException("writeEveryNTicks must be 1 or larger!");
      
      this.writeEveryNTicks = writeEveryNTicks;
      if (writeEveryNTicks != 1)
      {
         ticksTillNextWrite = new IntegerYoVariable("ticksTillNextWrite", registry);
         ticksTillNextWrite.set(0);
      }
      else
      {
         ticksTillNextWrite = null;
      }
      this.writeOnInitialize = writeOnInitialize;
   }

   @Override
   public void doControl()
   {
      if ((ticksTillNextWrite == null) || (ticksTillNextWrite.getIntegerValue() <= 0))
      {
         write();
         
         if (ticksTillNextWrite != null) ticksTillNextWrite.set(writeEveryNTicks-1);
      }
      else
      {
         ticksTillNextWrite.decrement();
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return "YoWhiteBoardWriteController";
   }

   @Override
   public void initialize()
   {
      if (writeOnInitialize)
      {
         write();
      }
   }

   @Override
   public void update()
   {
      doControl();
   }

   @Override
   public String getDescription()
   {
      return "YoWhiteBoardWriteController";
   }
   
   private void write()
   {
      try
      {
         yoWhiteBoard.writeData();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
