package us.ihmc.robotics.sensors;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.Wrench;

public interface ForceSensorDataHolderReadOnly
{
   public abstract ForceSensorDataReadOnly get(ForceSensorDefinition forceSensor);

   public abstract ForceSensorDataReadOnly getByName(String name);

   public abstract List<ForceSensorDefinition> getForceSensorDefinitions();

   public abstract void getForceSensorValue(ForceSensorDefinition key, Wrench wrenchToPack);

   public abstract void getForceSensorValue(ForceSensorDefinition key, DenseMatrix64F wrenchToPack);

   public abstract ForceSensorDefinition findForceSensorDefinition(String name);
}
