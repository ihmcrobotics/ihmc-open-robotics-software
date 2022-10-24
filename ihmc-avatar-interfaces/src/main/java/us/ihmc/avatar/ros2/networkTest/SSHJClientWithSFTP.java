package us.ihmc.avatar.ros2.networkTest;

import net.schmizz.sshj.SSHClient;
import net.schmizz.sshj.sftp.SFTPClient;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

public class SSHJClientWithSFTP extends SSHJClient
{
   private final SFTPClient sftpClient;

   public SSHJClientWithSFTP(SSHClient sshClient, SFTPClient sftpClient)
   {
      super(sshClient);
      this.sftpClient = sftpClient;
   }

   public void put(String source, String dest)
   {
      LogTools.info("Putting $source to ${ssh.remoteHostname}:$dest");
      ExceptionTools.handle(() -> sftpClient.put(source, dest), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void get(String source, String dest)
   {
      LogTools.info("Getting ${ssh.remoteHostname}:$source to $dest");
      ExceptionTools.handle(() -> sftpClient.get(source, dest), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }
}
