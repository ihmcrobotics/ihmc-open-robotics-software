package us.ihmc.simulationConstructionSetTools.whiteBoard;



import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotController.SensorProcessor;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.simulationconstructionset.util.IndexOrderChecker;

public class YoWhiteBoardReadController implements RobotController, SensorProcessor
{
   private final YoVariableRegistry registry;
   private final ExecutionTimer yoWhiteBoardReadControllerWaitingGlobalTimer;
   
   private final YoInteger numberOfNewDataSinceLastRead;
   private final YoInteger ticksTillNextRead;

   private final YoWhiteBoard yoWhiteBoard;
   private final boolean blockUntilNewDataIsAvailable;
   private final int readEveryNTicks;
   private final IndexOrderChecker indexOrderChecker;
   private final YoInteger previousMissedIndices;

   private final boolean readOnInitialize;

   public YoWhiteBoardReadController(String name, YoWhiteBoard yoWhiteBoard, boolean blockUntilNewDataIsAvailable, int readEveryNTicks,
                                     boolean doNotReadFirstTime, boolean readOnInitialize)
   {
      registry = new YoVariableRegistry(name + "YoWhiteBoardReadController");
      yoWhiteBoardReadControllerWaitingGlobalTimer = new ExecutionTimer("whiteBoardReadWait", registry);
      numberOfNewDataSinceLastRead = new YoInteger("numberOfNewDataSinceLastRead", registry);

      this.yoWhiteBoard = yoWhiteBoard;
      this.blockUntilNewDataIsAvailable = blockUntilNewDataIsAvailable;

      if (readEveryNTicks < 1)
         throw new RuntimeException("readEveryNTicks must be 1 or larger!");
      this.readEveryNTicks = readEveryNTicks;

      if (readEveryNTicks != 1)
      {
         ticksTillNextRead = new YoInteger("ticksTillNextRead", registry);

         if (doNotReadFirstTime)
         {
            ticksTillNextRead.set(readEveryNTicks - 1);
         }
         else
         {
            ticksTillNextRead.set(0);
         }
      }
      else
      {
         ticksTillNextRead = null;
         if (doNotReadFirstTime == true)
            throw new RuntimeException("doNotReadFirstTime must be false if readEveryNTicks == 1");
      }

      this.indexOrderChecker = new IndexOrderChecker(name, registry, 1);
      this.previousMissedIndices = new YoInteger("previousMissedIndices", registry);
      this.readOnInitialize = readOnInitialize;
   }

   @Override
   public void doControl()
   {
      if ((ticksTillNextRead == null) || (ticksTillNextRead.getIntegerValue() <= 0))
      {
         read(blockUntilNewDataIsAvailable);

         if (ticksTillNextRead != null)
            ticksTillNextRead.set(readEveryNTicks - 1);
      }
      else
      {
         ticksTillNextRead.decrement();
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
      return "YoWhiteBoardReadController";
   }

   @Override
   public void initialize()
   {
      if (readOnInitialize)
         read(true);
   }

   @Override
   public void update()
   {
      doControl();
   }

   @Override
   public String getDescription()
   {
      return "YoWhiteBoardReadController";
   }
   
   private void read(boolean block)
   {
      if (block)
      {
         synchronized (yoWhiteBoard)
         {
            while (!yoWhiteBoard.isNewDataAvailable())
            {
               yoWhiteBoardReadControllerWaitingGlobalTimer.startMeasurement();
               try
               {
                  yoWhiteBoard.wait();
               }
               catch (InterruptedException e)
               {
               }
               yoWhiteBoardReadControllerWaitingGlobalTimer.stopMeasurement();
            }
         }
      }

      numberOfNewDataSinceLastRead.set(yoWhiteBoard.getNumberOfNewDataSinceLastRead());
      yoWhiteBoard.readData();

      indexOrderChecker.update(yoWhiteBoard.getReadIndex());
      if (indexOrderChecker.getMissedIndices() - previousMissedIndices.getIntegerValue() > 0)
         System.out.println("YoWhiteBoardReadController: missed " + indexOrderChecker.getMissedIndices() + " YoWhiteBoard indices!");
      previousMissedIndices.set(indexOrderChecker.getMissedIndices());
   }
}
