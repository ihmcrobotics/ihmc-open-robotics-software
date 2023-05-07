package us.ihmc.rdx.simulation.scs2;

import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.robot.Robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * See {@link RestartableSCS2SessionRobotInterface}.
 */
public class RestartableSCS2SessionRobot implements RestartableSCS2SessionRobotInterface
{
   private final Supplier<RobotDefinition> robotDefinitionSupplier;
   private final Consumer<Robot> setupRobotConsumer;

   public RestartableSCS2SessionRobot(Supplier<RobotDefinition> robotDefinitionSupplier, Consumer<Robot> setupRobotConsumer)
   {
      this.robotDefinitionSupplier = robotDefinitionSupplier;
      this.setupRobotConsumer = setupRobotConsumer;
   }

   public Supplier<RobotDefinition> getRobotDefinitionSupplier()
   {
      return robotDefinitionSupplier;
   }

   public Consumer<Robot> getSetupRobotConsumer()
   {
      return setupRobotConsumer;
   }
}
