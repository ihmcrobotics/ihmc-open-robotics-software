package us.ihmc.communication.streamingData;

import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

public interface EstablishedAConnectionListener
{
   public abstract void establishedAConnection(ObjectInputStream objectInputStream, ObjectOutputStream objectOutputStream);
}
